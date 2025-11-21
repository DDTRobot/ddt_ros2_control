#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "inferrer_base.hpp"

// TensorRT 日志回调（恢复关键日志输出）
class TRTLogger : public nvinfer1::ILogger
{
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity == Severity::kERROR || severity == Severity::kINTERNAL_ERROR) {
      std::cerr << "TRT_ERROR: " << msg << std::endl;  // 输出错误和内部错误日志
    } else if (severity == Severity::kWARNING) {
      std::cout << "TRT_WARNING: " << msg << std::endl;  // 输出警告日志
    }
  }
};

class EngineInferrer : public InferrerBase
{
public:
  bool loadModel(const std::string & modelPath, bool verbose = false) override;
  std::vector<tensor_element_t> computeActions(
    const std::vector<std::vector<tensor_element_t>> & inputData) override;
  bool setOutput(const std::string & outputName, size_t outputSize) override;
  ~EngineInferrer();

private:
  // TensorRT 核心对象
  TRTLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  cudaStream_t stream_ = nullptr;  // CUDA流成员

  // 内存管理
  std::vector<void *> deviceBindings_;  // GPU输入输出内存指针（按模型绑定顺序）
  std::vector<void *> deviceInputs_;    // 所有输入的GPU内存指针（新增）
  std::vector<void *>
    deviceOutputs_;  // 所有输出的GPU内存指针（建议新增，替代原deviceBindings_直接操作）

  // 清理资源的辅助函数
  void freeResources();
};
