// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "inferrer/engine_inferrer.hpp"

// 加载TensorRT engine文件
bool EngineInferrer::loadModel(const std::string & modelPath, bool verbose)
{
  verbose_ = verbose;
  if (verbose_) {
    std::cout << "[EngineInferrer] Loading policy from: " << modelPath << std::endl;
  }
  freeResources();  // 释放旧资源
  // 1. 读取engine文件内容（不变）
  FILE * f = fopen(modelPath.c_str(), "rb");
  if (!f) {
    throw std::runtime_error(
      "[EngineInferrer] Failed to open engine file: " + modelPath);  // 抛出异常
  }

  fseek(f, 0, SEEK_END);
  size_t size = ftell(f);
  fseek(f, 0, SEEK_SET);
  std::vector<char> engineData(size);
  fread(engineData.data(), 1, size, f);
  fclose(f);

  // 2. 创建TensorRT运行时
  runtime_ = std::unique_ptr<nvinfer1::IRuntime>(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) {
    throw std::runtime_error("[EngineInferrer] Failed to create TensorRT runtime");  // 抛出异常
  }

  // 3. 反序列化engine
  engine_ = std::unique_ptr<nvinfer1::ICudaEngine>(
    runtime_->deserializeCudaEngine(engineData.data(), size, nullptr));
  if (!engine_) {
    throw std::runtime_error("[EngineInferrer] Failed to deserialize engine");  // 抛出异常
  }

  // 4. 创建执行上下文
  context_ = std::unique_ptr<nvinfer1::IExecutionContext>(engine_->createExecutionContext());
  if (!context_) {
    throw std::runtime_error("[EngineInferrer] Failed to create execution context");  // 抛出异常
  }

  // 5. 获取输入输出信息（多输入支持）
  for (int i = 0; i < engine_->getNbBindings(); ++i) {
    std::string name = engine_->getBindingName(i);
    nvinfer1::Dims dims = engine_->getBindingDimensions(i);
    if (engine_->bindingIsInput(i)) {
      inputNames_.push_back(name);
      size_t size = 1;
      std::vector<int64_t> shape;
      for (int d = 0; d < dims.nbDims; ++d) {
        size *= dims.d[d];
        shape.push_back(dims.d[d]);
      }
      inputShapes_.push_back(shape);
      inputSizes_.push_back(size);
    } else {
      outputNames_.push_back(name);
      size_t size = 1;
      std::vector<int64_t> shape;
      for (int d = 0; d < dims.nbDims; ++d) {
        size *= dims.d[d];
        shape.push_back(dims.d[d]);
      }
      outputShapes_.push_back(shape);
      outputSizes_.push_back(size);
    }
  }

  // 6. 分配CUDA内存（多输入支持）
  cudaStreamCreate(&stream_);

  // 分配所有输入的GPU内存
  for (size_t size : inputSizes_) {
    void * ptr = nullptr;
    cudaError_t cudaStatus = cudaMalloc(&ptr, size * sizeof(tensor_element_t));
    if (cudaStatus != cudaSuccess) {
      throw std::runtime_error(
        "[EngineInferrer] cudaMalloc input failed: " + std::string(cudaGetErrorString(cudaStatus)));
    }
    deviceInputs_.push_back(ptr);
  }

  // 分配所有输出的GPU内存（原有逻辑不变）
  for (size_t size : outputSizes_) {
    void * ptr = nullptr;
    cudaError_t cudaStatus = cudaMalloc(&ptr, size * sizeof(tensor_element_t));
    if (cudaStatus != cudaSuccess) {
      throw std::runtime_error(
        "[EngineInferrer] cudaMalloc output failed: " +
        std::string(cudaGetErrorString(cudaStatus)));
    }
    deviceOutputs_.push_back(ptr);
  }
  // 设置绑定顺序（所有输入 + 所有输出）
  deviceBindings_.clear();
  deviceBindings_.insert(deviceBindings_.end(), deviceInputs_.begin(), deviceInputs_.end());
  deviceBindings_.insert(deviceBindings_.end(), deviceOutputs_.begin(), deviceOutputs_.end());
  if (verbose_) {
    std::cout << "[EngineInferrer] Successfully loaded TensorRT engine: " << modelPath << std::endl;
    printModelInfo();
  }
  return true;
}

bool EngineInferrer::setOutput(const std::string & outputName, size_t outputSize)
{
  // 设置目标输出的GPU指针
  auto it = std::find(outputNames_.begin(), outputNames_.end(), outputName);
  if (it != outputNames_.end()) {
    nnOutputIndex_ = static_cast<int>(it - outputNames_.begin());
    if (outputSize != outputSizes_[nnOutputIndex_]) {
      throw std::runtime_error(
        "[EngineInferrer] Output size mismatch: " + std::to_string(outputSize) + " vs " +
        std::to_string(outputSizes_[nnOutputIndex_]) + "\n");
    }
    nnOutputSize_ = outputSizes_[nnOutputIndex_];
    if (verbose_) {
      std::cout << "[EngineInferrer] Successfully set output to \"" << outputName
                << "\" with size: " << nnOutputSize_ << std::endl;
    }
    return true;
  } else {
    std::string availableOutputs;
    for (const auto & name : outputNames_) {
      availableOutputs += (availableOutputs.empty() ? "" : ", ") + name;
    }
    throw std::runtime_error(
      "[EngineInferrer] Failed to find output: \"" + outputName +
      "\""
      ", Available outputs: " +
      availableOutputs);
    return false;
  }
}

// 执行推理计算（多输入支持）
std::vector<tensor_element_t> EngineInferrer::computeActions(
  const std::vector<std::vector<tensor_element_t>> & inputData)
{
  // 验证输入数量和大小
  if (inputData.size() != inputSizes_.size()) {
    throw std::runtime_error(
      "[EngineInferrer] Input count mismatch: " + std::to_string(inputData.size()) + " vs " +
      std::to_string(inputSizes_.size()) + "\n");
    return {};
  }
  for (size_t i = 0; i < inputData.size(); ++i) {
    if (inputData[i].size() != inputSizes_[i]) {
      throw std::runtime_error(
        "[EngineInferrer] Input size mismatch: " + std::to_string(inputData[i].size()) + " vs " +
        std::to_string(inputSizes_[i]) + "\n");
      return {};
    }
  }
  cudaError_t cudaStatus;
  // 拷贝每个输入到对应的GPU内存
  for (size_t i = 0; i < inputData.size(); ++i) {
    cudaStatus = cudaMemcpyAsync(
      deviceInputs_[i], inputData[i].data(), inputSizes_[i] * sizeof(tensor_element_t),
      cudaMemcpyHostToDevice, stream_);
    // std::cout << deviceInputs_[i] << std::endl;
    // std::cout << "uboyteData" << inputData[i].data()[11] << std::endl;
    // std::cout << "uboyteData" << inputSizes_[i]  << std::endl;

    if (cudaStatus != cudaSuccess) {
      throw std::runtime_error(
        "[EngineInferrer] cudaMemcpyAsync input " + std::to_string(i) +
        " failed: " + std::string(cudaGetErrorString(cudaStatus)));
    }
  }

  // 执行推理（不变）
  bool status = context_->enqueueV2(deviceBindings_.data(), stream_, nullptr);
  if (!status) {
    throw std::runtime_error("[EngineInferrer] Inference failed");
  }

  // 仅拷贝目标输出nn_output到CPU
  void * hostOutput_ = nullptr;
  cudaStatus = cudaMallocHost(&hostOutput_, nnOutputSize_ * sizeof(tensor_element_t));
  if (cudaStatus != cudaSuccess) {
    throw std::runtime_error(
      "[EngineInferrer] cudaMallocHost output failed: " +
      std::string(cudaGetErrorString(cudaStatus)));
  }

  cudaStatus = cudaMemcpyAsync(
    hostOutput_, deviceOutputs_[nnOutputIndex_], nnOutputSize_ * sizeof(tensor_element_t),
    cudaMemcpyDeviceToHost, stream_);
  if (cudaStatus != cudaSuccess) {
    throw std::runtime_error(
      "[EngineInferrer] cudaMemcpyAsync output failed: " +
      std::string(cudaGetErrorString(cudaStatus)));
    cudaFreeHost(hostOutput_);  // 释放临时分配的内存
    return {};
  }

  // 同步流（不变）
  cudaStatus = cudaStreamSynchronize(stream_);
  if (cudaStatus != cudaSuccess) {
    throw std::runtime_error(
      "[EngineInferrer] cudaStreamSynchronize failed: " +
      std::string(cudaGetErrorString(cudaStatus)));
    cudaFreeHost(hostOutput_);  // 释放临时分配的内存
    return {};
  }

  // 生成结果（不变）
  auto result = std::vector<tensor_element_t>(
    static_cast<tensor_element_t *>(hostOutput_),
    static_cast<tensor_element_t *>(hostOutput_) + nnOutputSize_);
  cudaFreeHost(hostOutput_);  // 释放临时CPU内存

  return result;
}

// 资源释放（多输入支持）
void EngineInferrer::freeResources()
{
  // 释放所有输入的GPU内存
  for (void * ptr : deviceInputs_) {
    cudaFree(ptr);
  }
  // 释放所有输出的GPU内存
  if (!deviceBindings_.empty()) {
    for (size_t i = 1; i < deviceBindings_.size(); ++i) {  // 第一个是输入，后面是输出
      cudaFree(deviceBindings_[i]);
    }
    deviceBindings_.clear();
  }
  if (stream_ != nullptr) {
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
  context_.reset();
  engine_.reset();
  runtime_.reset();
}

EngineInferrer::~EngineInferrer() { freeResources(); }