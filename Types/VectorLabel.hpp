#ifndef LEPH_VECTORLABEL_HPP
#define LEPH_VECTORLABEL_HPP

#include <vector>
#include <map>
#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <sstream>
#include "Types/types.h"

namespace Leph
{
/**
 * VectorLabel
 *
 * Associate each Eigen vector element
 * with a string label
 */
class VectorLabel
{
public:
  /**
   * Mapping container typedef
   */
  typedef std::map<std::string, size_t> LabelContainer;
  typedef std::vector<std::string> IndexContainer;
  typedef std::vector<std::string> LabelList;

  /**
   * Initialization
   * If label are not given, default labes
   * are used
   */
  VectorLabel() : _eigenVector(), _labelToIndex(nullptr), _indexToLabel(nullptr)
  {
    _labelToIndex = std::make_shared<LabelContainer>();
    _indexToLabel = std::make_shared<IndexContainer>();
  }
  VectorLabel(size_t size) : _eigenVector(Vector::Zero(size)), _labelToIndex(nullptr), _indexToLabel(nullptr)
  {
    _labelToIndex = std::make_shared<LabelContainer>();
    _indexToLabel = std::make_shared<IndexContainer>();
    defaultLabels();
  }

  VectorLabel(const LabelList& labels)
    : _eigenVector(Vector::Zero(labels.size())), _labelToIndex(nullptr), _indexToLabel(nullptr)
  {
    _labelToIndex = std::make_shared<LabelContainer>();
    _indexToLabel = std::make_shared<IndexContainer>(labels);
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      if (_labelToIndex->count((*_indexToLabel)[i]) != 0)
      {
        throw std::logic_error("VectorLabel label error");
      }
      (*_labelToIndex)[(*_indexToLabel)[i]] = i;
    }
  }
  VectorLabel(const Vector& vect) : _eigenVector(vect), _labelToIndex(nullptr), _indexToLabel(nullptr)
  {
    _labelToIndex = std::make_shared<LabelContainer>();
    _indexToLabel = std::make_shared<IndexContainer>();
    defaultLabels();
  }
  VectorLabel(const LabelList& labels, const Vector& vect)
    : _eigenVector(vect), _labelToIndex(nullptr), _indexToLabel(nullptr)
  {
    _labelToIndex = std::make_shared<LabelContainer>();
    _indexToLabel = std::make_shared<IndexContainer>(labels);
    if (labels.size() != (size_t)vect.size())
    {
      throw std::logic_error("VectorLabel size invalid");
    }
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      if (_labelToIndex->count((*_indexToLabel)[i]) != 0)
      {
        throw std::logic_error("VectorLabel label error");
      }
      (*_labelToIndex)[(*_indexToLabel)[i]] = i;
    }
  }

  /**
   * Variadic template label, value
   * mapping initialization
   */
  VectorLabel(const std::string& label, double value) : _eigenVector(), _labelToIndex(nullptr), _indexToLabel(nullptr)
  {
    _labelToIndex = std::make_shared<LabelContainer>();
    _indexToLabel = std::make_shared<IndexContainer>();
    appendAux(label, value);
  }
  template <class... LabelsValues>
  VectorLabel(const std::string& label, double value, LabelsValues... labelsValues)
    : _eigenVector(), _labelToIndex(nullptr), _indexToLabel(nullptr)
  {
    _labelToIndex = std::make_shared<LabelContainer>();
    _indexToLabel = std::make_shared<IndexContainer>();
    appendAux(label, value);
    append(labelsValues...);
  }

  /**
   * Append given label and optional value
   * to the vector container
   */
  inline void append(const std::string& label, double value = 0.0)
  {
    appendAux(label, value);
  }
  template <class... LabelsValues>
  inline void append(const std::string& label, double value, LabelsValues... labelsValues)
  {
    appendAux(label, value);
    append(labelsValues...);
  }

  /**
   * Set value if given label exists or
   * append it
   */
  inline void setOrAppend(const std::string& name, double value)
  {
    if (_labelToIndex->count(name) == 0)
    {
      append(name, value);
    }
    else
    {
      operator()(name) = value;
    }
  }

  /**
   * Direct access to Eigen vector
   */
  inline const Vector& vect() const
  {
    return _eigenVector;
  }
  inline Vector& vect()
  {
    return _eigenVector;
  }

  /**
   * Direct access to labels container
   */
  inline const LabelContainer& labels() const
  {
    return *_labelToIndex;
  }

  /**
   * Return the vector size
   * (Optionaly filtered with given section)
   */
  inline size_t size(const std::string& filter = "") const
  {
    if (filter == "")
    {
      return _labelToIndex->size();
    }
    else
    {
      size_t sum = 0;
      for (size_t i = 0; i < _indexToLabel->size(); i++)
      {
        if (toSection(_indexToLabel->at(i)) == filter)
        {
          sum++;
        }
      }
      return sum;
    }
  }

  /**
   * Mapping from label to index and back
   */
  inline const std::string& getLabel(size_t index) const
  {
    if (index >= size())
    {
      throw std::logic_error("VectorLabel unbound index");
    }

    return _indexToLabel->at(index);
  }
  inline size_t getIndex(const std::string& label) const
  {
    return _labelToIndex->at(label);
  }

  /**
   * Return true if the given labels
   * is registered
   */
  inline bool exist(const std::string& label) const
  {
    return (_labelToIndex->count(label) > 0);
  }

  /**
   * Access to named vector element
   */
  inline const double& operator()(const std::string& label) const
  {
    size_t index;
    try
    {
      index = _labelToIndex->at(label);
    }
    catch (const std::out_of_range& err)
    {
      throw std::logic_error("VectorLabel invalid label: " + label);
    }

    return _eigenVector(index);
  }
  inline double& operator()(const std::string& label)
  {
    size_t index;
    try
    {
      index = _labelToIndex->at(label);
    }
    catch (const std::out_of_range& err)
    {
      throw std::logic_error("VectorLabel invalid label: " + label);
    }

    return _eigenVector(index);
  }

  /**
   * Access to indexed element
   */
  inline const double& operator()(size_t index) const
  {
    return _eigenVector(index);
  }
  inline double& operator()(size_t index)
  {
    return _eigenVector(index);
  }

  /**
   * Merge the given VectorLabel to this
   * Value priority to given other
   * Result labels are the union of given
   * (filtered with given section) vector and this
   * (All (filtered) values in given vector are created
   * or update in this)
   */
  inline void mergeUnion(const VectorLabel& v, const std::string& filter = "")
  {
    for (const auto& label : v.labels())
    {
      if (filter == "" || toSection(label.first) == filter)
      {
        if (!exist(label.first))
        {
          appendAux(label.first, v(label.first));
        }
        else
        {
          operator()(label.first) = v(label.first);
        }
      }
    }
  }

  /**
   * Merge the given VectorLabel to this
   * Value priority to given other
   * Result labels are the intersection of given
   * (filtered with given section) vector and this and
   * others labels of this.
   * (Only values present in this and given (filtered) vector
   * are updated. Others are not update either deleted)
   */
  inline void mergeInter(const VectorLabel& v, const std::string& filter = "")
  {
    for (const auto& label : v.labels())
    {
      if (filter == "" || toSection(label.first) == filter)
      {
        if (exist(label.first))
        {
          operator()(label.first) = v(label.first);
        }
      }
    }
  }

  /**
   * Merge the two given VectorLabel and concat
   * them (value priority to first parameter)
   * Result labels are the union of given vectors labels
   */
  static inline VectorLabel mergeUnion(const VectorLabel& v1, const VectorLabel& v2)
  {
    VectorLabel merged = v1;
    for (const auto& label : v2.labels())
    {
      if (!merged.exist(label.first))
      {
        merged.appendAux(label.first, v2(label.first));
      }
    }

    return merged;
  }

  /**
   * Merge the two given VectorLabel
   * (value priority to first parameter)
   * Result labels are the intersection of given vectors labels
   */
  static inline VectorLabel mergeInter(const VectorLabel& v1, const VectorLabel& v2)
  {
    VectorLabel merged;
    for (const auto& label : v1.labels())
    {
      if (v2.exist(label.first))
      {
        merged.appendAux(label.first, v1(label.first));
      }
    }

    return merged;
  }

  /**
   * Print utility
   */
  inline void print(std::ostream& os = std::cout) const
  {
    unsigned int maxLength = 0;
    unsigned int maxDigit = _indexToLabel->size() > 10 ? 2 : 1;
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      if (_indexToLabel->at(i).length() > maxLength)
      {
        maxLength = _indexToLabel->at(i).length();
      }
    }
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      os << "[" << std::left << std::setw(maxDigit) << i << ":" << std::left << std::setw(maxLength)
         << _indexToLabel->at(i) << "]"
         << " " << std::setprecision(17) << _eigenVector(i) << std::endl;
    }
  }

  /**
   * Write values in CSV format to given stream
   */
  inline void writeToCSV(std::ostream& os = std::cout) const
  {
    os << "# ";
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      os << "'" << _indexToLabel->at(i) << "' ";
    }
    os << std::endl;
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      os << std::setprecision(17) << _eigenVector(i) << " ";
    }
    os << std::endl;
  }

  /**
   * Read and load values in CSV format from
   * given string
   */
  inline void readFromCSV(const std::string& str)
  {
    // Skip empty input
    if (str.length() == 0)
      return;
    // Check labels line is commented
    if (str[0] != '#')
      throw std::logic_error("VectorLabel invalid CSV format (first comment line): " + str);
    // Find first label
    size_t index = 0;
    index = str.find_first_of(std::string("'"), index);
    if (index == std::string::npos)
      throw std::logic_error("VectorLabel invalid CSV format (no label): " + str);
    // Init extracted labels and CSV index mapping
    std::map<size_t, std::string> mapping;
    size_t labelIndex = 0;
    // Extract all labels until newline
    while (index != std::string::npos && str[index] != '\n')
    {
      size_t endLabel = str.find_first_of(std::string("'\n"), index + 1);
      std::string label = str.substr(index + 1, endLabel - index - 1);
      if (!exist(label))
      {
        append(label, 0.0);
      }
      mapping[labelIndex] = label;
      index = str.find_first_of(std::string("'\n"), endLabel + 1);
      labelIndex++;
    }
    // Go through new line
    index = str.find_first_not_of(std::string("' \n"), index);
    // Extract all values
    labelIndex = 0;
    while (index != std::string::npos && str[index] != '\n')
    {
      size_t endLabel = str.find_first_of(std::string(" \n"), index);
      std::string value = str.substr(index, endLabel - index);
      if (mapping.count(labelIndex) == 0)
        throw std::logic_error("VectorLabel invalid CSV format (mismatch values labels): " + str);
      operator()(mapping.at(labelIndex)) = std::atof(value.c_str());
      index = str.find_first_not_of(std::string(" "), endLabel);
      labelIndex++;
    }
    // Check labels and values match
    if (labelIndex != mapping.size())
      throw std::logic_error("VectorLabel invalid CSV format (mismatch values labels): " + str);
  }

  /**
   * Read one vector label data (two lines) from
   * given stream.
   * Return false when given stream is not good anymore
   * else return true.
   */
  inline bool readFromCSV(std::istream& is)
  {
    std::string line1;
    std::string line2;
    getline(is, line1);
    getline(is, line2);

    if (line1.length() != 0 || line2.length() != 0)
    {
      readFromCSV(line1 + "\n" + line2);
    }

    return is.good();
  }

  /**
   * Return a sub VectorLabel with only labels
   * filtered with given section
   */
  inline VectorLabel extract(const std::string& filter) const
  {
    VectorLabel tmp;
    tmp.mergeUnion(*this, filter);

    return tmp;
  }

  /**
   * Return a VectorLabel with all labels matching
   * filterSrc rename to filterDst section name
   */
  inline VectorLabel rename(const std::string& filterSrc, const std::string& filterDst) const
  {
    VectorLabel tmp;
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      const std::string& label = _indexToLabel->at(i);
      std::string dstLabel = filterDst + ":" + toName(label);
      if (toSection(label) == filterSrc)
      {
        if (filterDst == "")
        {
          dstLabel = toName(label);
        }
        else
        {
          dstLabel = filterDst + ":" + toName(label);
        }
      }
      else
      {
        dstLabel = label;
      }
      if (!tmp.exist(dstLabel))
      {
        tmp.appendAux(dstLabel, _eigenVector(i));
      }
      else
      {
        tmp(dstLabel) = _eigenVector(i);
      }
    }

    return tmp;
  }

  /**
   * Apply the operation "func" on all labels of given VectorLabel
   * filtered by filterSrc section and write the result on either
   * same label in this or filteredDst section and same name
   */
  inline void op(const VectorLabel& vect, std::function<void(double& self, const double& other)> func,
                 const std::string& filterSrc = "#", const std::string& filterDst = "#")
  {
    for (size_t i = 0; i < vect._indexToLabel->size(); i++)
    {
      const std::string& srcLabel = vect._indexToLabel->at(i);
      std::string dstLabel;
      if (filterDst != "#")
      {
        if (filterDst == "")
        {
          dstLabel = toName(srcLabel);
        }
        else
        {
          dstLabel = filterDst + ":" + toName(srcLabel);
        }
      }
      else
      {
        dstLabel = srcLabel;
      }

      if (exist(dstLabel) && (filterSrc == "#" || toSection(srcLabel) == filterSrc))
      {
        size_t index = _labelToIndex->at(dstLabel);
        func(_eigenVector(index), vect._eigenVector(i));
      }
    }
  }

  /**
   * Apply the operation "func" with scalar argument value
   * on all labels of this VectorLabel filtered by filter
   */
  inline void op(double value, std::function<void(double& self, double value)> func, const std::string& filter = "#")
  {
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      const std::string& label = _indexToLabel->at(i);
      if ((filter == "#" || toSection(label) == filter))
      {
        func(_eigenVector(i), value);
      }
    }
  }

  /**
   * Apply the unary operation "func" on all labels of
   * this VectorLabel filtered by filter
   */
  inline void op(std::function<void(double& self)> func, const std::string& filter = "#")
  {
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      const std::string& label = _indexToLabel->at(i);
      if ((filter == "#" || toSection(label) == filter))
      {
        func(_eigenVector(i));
      }
    }
  }

  /**
   * Coefficient wise addition, substraction, multiplication
   * and scalar multiplication with optional source and destination
   * section filter
   */
  inline void addOp(const VectorLabel& vect, const std::string& filterSrc = "#", const std::string& filterDst = "#")
  {
    op(vect, [](double& self, const double& other) { self += other; }, filterSrc, filterDst);
  }
  inline void subOp(const VectorLabel& vect, const std::string& filterSrc = "#", const std::string& filterDst = "#")
  {
    op(vect, [](double& self, const double& other) { self -= other; }, filterSrc, filterDst);
  }
  inline void mulOp(const VectorLabel& vect, const std::string& filterSrc = "#", const std::string& filterDst = "#")
  {
    op(vect, [](double& self, const double& other) { self *= other; }, filterSrc, filterDst);
  }
  inline void divOp(const VectorLabel& vect, const std::string& filterSrc = "#", const std::string& filterDst = "#")
  {
    op(vect, [](double& self, const double& other) { self /= other; }, filterSrc, filterDst);
  }
  inline void assignOp(const VectorLabel& vect, const std::string& filterSrc = "#", const std::string& filterDst = "#")
  {
    op(vect, [](double& self, const double& other) { self = other; }, filterSrc, filterDst);
  }
  inline void addOp(double val, const std::string& filter = "#")
  {
    op(val, [](double& self, double value) { self += value; }, filter);
  }
  inline void subOp(double val, const std::string& filter = "#")
  {
    op(val, [](double& self, double value) { self -= value; }, filter);
  }
  inline void mulOp(double val, const std::string& filter = "#")
  {
    op(val, [](double& self, double value) { self *= value; }, filter);
  }
  inline void divOp(double val, const std::string& filter = "#")
  {
    op(val, [](double& self, double value) { self /= value; }, filter);
  }
  inline void powerOp(double val, const std::string& filter = "#")
  {
    op(val, [](double& self, double value) { self = pow(self, value); }, filter);
  }
  inline void squareOp(const std::string& filter = "#")
  {
    op([](double& self) { self *= self; }, filter);
  }
  inline void sqrtOp(const std::string& filter = "#")
  {
    op([](double& self) { self = sqrt(self); }, filter);
  }
  inline void zeroOp(const std::string& filter = "#")
  {
    op([](double& self) { self = 0.0; }, filter);
  }
  inline void minOp(double val, const std::string& filter = "#")
  {
    op(val,
       [](double& self, double value) {
         if (self < value)
           self = value;
       },
       filter);
  }
  inline void maxOp(double val, const std::string& filter = "#")
  {
    op(val,
       [](double& self, double value) {
         if (self > value)
           self = value;
       },
       filter);
  }

  /**
   * Return the mean of (filtered)
   * contained values
   */
  inline double mean(const std::string& filter = "#")
  {
    double sum = 0.0;
    int count = 0;
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      const std::string& label = _indexToLabel->at(i);
      if ((filter == "#" || toSection(label) == filter))
      {
        sum += _eigenVector(i);
        count++;
      }
    }

    if (count > 0)
    {
      sum /= (double)count;
    }
    return sum;
  }

  /**
   * Return the sum of (filtered)
   * contained values
   */
  inline double sum(const std::string& filter = "#")
  {
    double sum = 0.0;
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      const std::string& label = _indexToLabel->at(i);
      if ((filter == "#" || toSection(label) == filter))
      {
        sum += _eigenVector(i);
      }
    }

    return sum;
  }

  /**
   * Return true one container value is NaN
   */
  inline bool isNan() const
  {
    for (size_t i = 0; i < _indexToLabel->size(); i++)
    {
      if (std::isnan(_eigenVector(i)) || std::isinf(_eigenVector(i)))
      {
        return true;
      }
    }
    return false;
  }

  /**
   * Return the name part and section part from
   * given string label (separator is ":")
   */
  static inline std::string toName(const std::string& label)
  {
    size_t index = label.find_first_of(std::string(":"));
    if (index != std::string::npos)
    {
      return label.substr(index + 1);
    }
    else
    {
      return label;
    }
  }
  static inline std::string toSection(const std::string& label)
  {
    size_t index = label.find_first_of(std::string(":"));
    if (index != std::string::npos)
    {
      return label.substr(0, index);
    }
    else
    {
      return "";
    }
  }

  /**
   * Return true if all (filtered) labels
   * in vect1 equals existing same label in vect2
   * (Condition threshold can given)
   */
  static inline bool isEqualInter(const VectorLabel& vect1, const VectorLabel& vect2, const std::string& filter = "",
                                  double threshold = 0.000001)
  {
    bool equals = true;
    for (size_t i = 0; i < vect1._indexToLabel->size(); i++)
    {
      const std::string& label = vect1._indexToLabel->at(i);
      if (vect2.exist(label) && (filter == "" || toSection(label) == filter) &&
          fabs(vect1(label) - vect2(label)) > threshold)
      {
        equals = false;
        break;
      }
    }

    return equals;
  }

private:
  /**
   * Double dynamic Eigen values
   * container vector
   */
  Vector _eigenVector;

  /**
   * String label to index mapping
   * container
   */
  std::shared_ptr<LabelContainer> _labelToIndex;

  /**
   * Index to string label mapping
   * container
   */
  std::shared_ptr<IndexContainer> _indexToLabel;

  /**
   * Build up default label
   */
  inline void defaultLabels()
  {
    for (size_t i = 0; i < (size_t)_eigenVector.size(); i++)
    {
      std::ostringstream oss;
      oss << "label " << i;
      (*_labelToIndex)[oss.str()] = i;
      _indexToLabel->push_back(oss.str());
    }
  }

  /**
   * Append implementation
   */
  inline void appendAux(const std::string& label, double value)
  {
    if (_labelToIndex->count(label) != 0)
    {
      throw std::logic_error("VectorLabel label already exists");
    }
    // Copy labels mapping container on modification
    _labelToIndex = std::make_shared<LabelContainer>(*_labelToIndex);
    _indexToLabel = std::make_shared<IndexContainer>(*_indexToLabel);
    // Then append a new
    size_t len = size();
    _eigenVector.conservativeResize(len + 1, Eigen::NoChange_t());
    _eigenVector(len) = value;
    (*_labelToIndex)[label] = len;
    _indexToLabel->push_back(label);
  }
};

/**
 * Overload stream operator
 */
inline std::ostream& operator<<(std::ostream& os, const VectorLabel& vect)
{
  vect.print(os);
  return os;
}

}  // namespace Leph

#endif
