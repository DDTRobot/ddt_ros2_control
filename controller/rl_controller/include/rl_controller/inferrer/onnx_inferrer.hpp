#include <onnxruntime_cxx_api.h>  // ONNX Runtime C++ API 头文件

#include <iostream>
#include <vector>

#include "inferrer_base.hpp"

class ONNXInferrer : public InferrerBase
{
public:
  bool loadModel(const std::string & modelPath, bool verbose = false) override;
  std::vector<tensor_element_t> computeActions(
    const std::vector<std::vector<tensor_element_t>> & inputData) override;
  bool setOutput(const std::string & outputName, size_t outputSize) override;

private:
  std::unique_ptr<Ort::Env> onnxEnvPrt_;
  std::unique_ptr<Ort::Session> policySessionPtr_;
};
