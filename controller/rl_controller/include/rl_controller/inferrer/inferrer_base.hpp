#ifndef INFERRED_BASE_HPP
#define INFERRED_BASE_HPP

#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using tensor_element_t = float;  // 保持与原项目一致的元素类型

class InferrerBase
{
public:
  // 纯虚函数：加载模型（由子类实现）
  virtual bool loadModel(const std::string & modelPath, bool verbose = false) = 0;

  // 纯虚函数：执行推理（由子类实现）
  virtual std::vector<tensor_element_t> computeActions(
    const std::vector<std::vector<tensor_element_t>> & inputData) = 0;
  virtual bool setOutput(const std::string & outputName, size_t outputSize) = 0;

  void printModelInfo()
  {
    std::cout << "===================== Model Infos ====================" << std::endl;
    for (size_t i = 0; i < inputNames_.size(); i++) {
      std::cout << "\tInputName:" << inputNames_[i] << "\tShape: ";
      for (size_t j = 0; j < inputShapes_[i].size(); j++) {
        std::cout << inputShapes_[i][j] << " ";
      }

      std::cout << "\tSize: " << inputSizes_[i] << " ";
      std::cout << std::endl;
    }
    for (size_t i = 0; i < outputNames_.size(); i++) {
      std::cout << "\tOutputName:" << outputNames_[i] << "\tShape: ";
      for (size_t j = 0; j < outputShapes_[i].size(); j++) {
        std::cout << outputShapes_[i][j] << " ";
      }
      std::cout << "\tSize: " << outputSizes_[i] << " ";
      std::cout << std::endl;
    }
  };

  // 公共接口：获取输入输出信息
  const std::vector<std::string> & getInputNames() const { return inputNames_; }
  const std::vector<std::string> & getOutputNames() const { return outputNames_; }
  const std::vector<std::vector<int64_t>> & getInputShapes() const { return inputShapes_; }
  const std::vector<std::vector<int64_t>> & getOutputShapes() const { return outputShapes_; }

protected:
  // 输入输出元数据（由子类填充）
  std::vector<std::string> inputNames_, outputNames_;
  std::vector<std::vector<int64_t>> inputShapes_,
    outputShapes_;                                // 每个输入输出的形状（如 [1, 420]）
  std::vector<size_t> inputSizes_, outputSizes_;  // 所有输入的总元素数（新增）
  size_t nnOutputSize_ = -1;                      // 目标输出nn_output的总元素数
  size_t nnOutputIndex_ = -1;                     // 目标输出nn_output的索引

  bool verbose_ = false;
};

#endif  // INFERRED_BASE_HPP
