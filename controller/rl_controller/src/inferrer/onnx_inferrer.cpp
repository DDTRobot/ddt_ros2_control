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

#include "rl_controller/inferrer/onnx_inferrer.hpp"

bool ONNXInferrer::loadModel(const std::string & modelPath, bool verbose)
{
  verbose_ = verbose;
  // Create ONNX environment
  onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "OnnxInferrer"));

  // Create session options
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetIntraOpNumThreads(1);
  sessionOptions.SetInterOpNumThreads(1);

  Ort::AllocatorWithDefaultOptions allocator;

  // Policy session
  try {
    policySessionPtr_ =
      std::make_unique<Ort::Session>(*onnxEnvPrt_, modelPath.c_str(), sessionOptions);
    if (verbose_) {
      std::cout << "Loading policy from: " << modelPath << std::endl;
    }
  } catch (const Ort::Exception & e) {
    throw std::runtime_error("Failed to load policy model: " + std::string(e.what()));
  }
  inputShapes_.clear();
  outputShapes_.clear();
  // 遍历输入（将const char*转换为std::string存入基类inputNames_）
  for (size_t i = 0; i < policySessionPtr_->GetInputCount(); i++) {
    const char * inputName = policySessionPtr_->GetInputName(i, allocator);  // 获取const char*
    inputNames_.push_back(std::string(inputName));  // 转换为std::string存入基类
    inputShapes_.push_back(
      policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    size_t size = 1;
    for (auto dim : inputShapes_[i]) {
      size *= static_cast<size_t>(dim);
    }
    inputSizes_.push_back(size);
  }
  // 遍历输出（同理）
  for (size_t i = 0; i < policySessionPtr_->GetOutputCount(); i++) {
    const char * outputName = policySessionPtr_->GetOutputName(i, allocator);  // 获取const char*
    outputNames_.push_back(std::string(outputName));  // 转换为std::string存入基类
    outputShapes_.push_back(
      policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
    size_t size = 1;
    for (auto dim : outputShapes_[i]) {
      size *= static_cast<size_t>(dim);
    }
    outputSizes_.push_back(size);
  }
  if (verbose_) {
    std::cout << "Successfully loaded ONNX models!" << std::endl;
    printModelInfo();
  }
  return true;
}
bool ONNXInferrer::setOutput(const std::string & outputName, size_t outputSize)
{
  auto it = std::find(outputNames_.begin(), outputNames_.end(), outputName);
  if (it != outputNames_.end()) {
    nnOutputIndex_ = static_cast<int>(it - outputNames_.begin());
    if (outputSize != outputSizes_[nnOutputIndex_]) {
      throw std::runtime_error(
        "[OnnxInferrer] Output size mismatch: " + std::to_string(outputSize) + " vs " +
        std::to_string(outputSizes_[nnOutputIndex_]) + "\n");
    }
    nnOutputSize_ = outputSizes_[nnOutputIndex_];
    if (verbose_) {
      std::cout << "[OnnxInferrer] Successfully set output to \"" << outputName
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
  return true;
}

// 执行推理计算（支持多输入）
std::vector<tensor_element_t> ONNXInferrer::computeActions(
  const std::vector<std::vector<tensor_element_t>> & inputData)
{
  // 验证输入数量与模型输入数量是否匹配
  if (inputData.size() != inputNames_.size()) {
    throw std::runtime_error(
      "Input count mismatch: " + std::to_string(inputData.size()) + " vs " +
      std::to_string(inputNames_.size()));
    // std::cerr << "Input count mismatch: " << inputData.size() << " vs " << inputNames_.size() << std::endl;
    // return {};
  }

  // 创建内存信息（CPU）
  Ort::MemoryInfo memoryInfo =
    Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

  // 为每个输入创建张量（关键修改：添加const_cast）
  std::vector<Ort::Value> inputValues;
  for (size_t i = 0; i < inputData.size(); ++i) {
    size_t size = 1;
    for (auto dim : inputShapes_[i]) {
      size *= static_cast<size_t>(dim);
    }
    if (inputData[i].size() != size) {
      throw std::runtime_error(
        "Input size mismatch: " + std::to_string(inputData[i].size()) + " vs " +
        std::to_string(size));
      // std::cerr << "Input " << i << " size mismatch: " << inputData[i].size() << " vs " << inputSizes_[i] << std::endl;
      // return {};
    }

    // 创建输入张量（使用const_cast移除const限定）
    inputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(
      memoryInfo,
      const_cast<tensor_element_t *>(inputData[i].data()),  // 关键修改点
      inputData[i].size(), inputShapes_[i].data(), inputShapes_[i].size()));
  }
  // 执行推理（关键修改：传入所有输入张量）
  Ort::RunOptions runOptions;
  std::vector<const char *> inputNames;
  for (const auto & name : inputNames_) {
    inputNames.push_back(name.c_str());  // std::string转const char*
  }
  // 转换输出名称为const char*数组（关键修改）
  std::vector<const char *> outputNames;
  for (const auto & name : outputNames_) {
    outputNames.push_back(name.c_str());  // std::string转const char*
  }
  std::vector<Ort::Value> outputValues = policySessionPtr_->Run(
    runOptions, inputNames.data(), inputValues.data(),
    static_cast<int>(inputValues.size()),  // 输入数量为inputValues的大小
    outputNames.data(),
    static_cast<int>(outputNames.size())  // 输出数量（假设取所有输出）
  );

  // 提取输出数据（假设取第一个输出，可根据需求调整）
  // nnOutputSize_ = outputValues[0].GetTensorTypeAndShapeInfo().GetElementCount();
  std::vector<tensor_element_t> actions(nnOutputSize_);
  tensor_element_t * outputData =
    outputValues[nnOutputIndex_].GetTensorMutableData<tensor_element_t>();
  std::copy(outputData, outputData + nnOutputSize_, actions.begin());

  return actions;
}