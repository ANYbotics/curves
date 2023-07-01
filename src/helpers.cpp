/*
 * @file helpers.hpp
 * @date Oct 17, 2014
 * @author Sean Andersson, Peter Fankhauser
 */

#include <fstream>

#include <curves/helpers.hpp>

namespace curves {

template <typename T>
std::string toString(const T& t) {
  std::ostringstream ss;
  ss << t;
  return ss.str();
}

/// \brief Helper function to read CSV files into 'matrix' of strings
std::vector<std::vector<std::string> > loadCSV(std::string fileName) {
  // Open file stream and check that it is error free
  std::ifstream inFileStream(fileName.c_str());
  assert(inFileStream.good() && "Error opening input file.");
  // Create output variable
  std::vector<std::vector<std::string> > strMatrix;
  // Loop over lines (rows) of CSV file
  std::string line;
  while (std::getline(inFileStream, line)) {
    // Make string stream out of line
    std::istringstream ss(line);
    std::vector<std::string> strRow;
    // Loop over comma separated fields of CSV line
    std::string field;
    while (getline(ss, field, ',')) {
      strRow.push_back(field);  // add column entry to row
    }
    strMatrix.push_back(strRow);  // add row to matrix
  }
  inFileStream.close();
  return strMatrix;
}

/// \brief Helper function to write 'matrix' of strings into CSV file
void writeCSV(std::string fileName, const std::vector<std::vector<std::string> >& strMatrix) {
  assert(strMatrix.size() >= 1 && "Provided matrix of strings has no entries.");
  std::ofstream outFileStream;
  outFileStream.open(fileName.c_str());
  // Iterate over the rows of the string matrix and write comma-separated fields
  for (const auto& itRow : strMatrix) {
    const unsigned fields = itRow.size();
    assert(fields >= 1 && "String matrix row has no entries.");
    outFileStream << itRow.at(0);
    for (unsigned i = 1; i < fields; i++) {
      outFileStream << "," << itRow.at(i);
    }
    outFileStream << std::endl;
  }
  outFileStream.close();
}

/// \brief Helper function to read CSV files formatted in: time, vectorEntry0, vectorEntry1, ...
void loadTimeVectorCSV(std::string fileName, std::vector<curves::Time>* outTimes, std::vector<Eigen::VectorXd>* outValues) {
  // Initialize outputs
  assert(outTimes != nullptr);
  outTimes->clear();
  assert(outValues != nullptr);
  outValues->clear();
  // Load CSV to matrix of strings
  std::vector<std::vector<std::string> > temp = loadCSV(fileName);
  assert(temp.size() >= 1 && "CSV file is empty.");
  assert(temp.at(0).size() >= 2 && "CSV file does not have the expected number of fields (atleast time and 1 value).");
  // Iterate over the rows of the CSV and use comma-separated fields to popular outputs
  const unsigned vOffset = 1;
  const unsigned vDim = temp.at(0).size() - vOffset;
  Eigen::VectorXd tempVec(vDim);
  for (const auto& it : temp) {
    outTimes->push_back((curves::Time)atof(it.at(1).c_str()));
    for (unsigned vIdx = 0; vIdx < vDim; vIdx++) {
      tempVec[vIdx] = atof(it.at(vIdx + vOffset).c_str());
    }
    outValues->push_back(tempVec);
  }
}

/// \brief Helper function to write CSV file formatted in: time, vectorEntry0, vectorEntry1, ...
void writeTimeVectorCSV(std::string fileName, const std::vector<curves::Time>& times, const std::vector<Eigen::VectorXd>& values) {
  // Check inputs and initialize sizes
  assert(times.size() == values.size() && "Length of times and values is not equal.");
  assert(times.size() == 1 && "No entries to write.");
  const unsigned vOffset = 1;
  const unsigned vDim = values.at(0).rows();
  // Fill string matrix
  std::vector<std::vector<std::string> > strMatrix;
  strMatrix.reserve(times.size());
  std::vector<std::string> strRow;
  strRow.reserve(vOffset + vDim);
  for (unsigned i = 0; i < times.size(); i++) {
    assert(vDim == values.at(i).rows() && "all vectors must be of the same dimension.");
    strRow.clear();
    strRow.push_back(toString<Time>(times[i]));
    for (unsigned v = 0; v < vDim; v++) {
      strRow.push_back(toString<double>(values[i][v]));
    };
    strMatrix.push_back(strRow);
  }
  // Write
  writeCSV(fileName, strMatrix);
}

/// \brief Helper function to read CSV files formatted in: time0, time1, vectorEntry0, vectorEntry1, ...
void loadTimeTimeVectorCSV(std::string fileName, std::vector<curves::Time>* outTimes0, std::vector<curves::Time>* outTimes1,
                           std::vector<Eigen::VectorXd>* outValues) {
  // Initialize outputs
  assert(outTimes0 != nullptr);
  outTimes0->clear();
  assert(outTimes1 != nullptr);
  outTimes1->clear();
  assert(outValues != nullptr);
  outValues->clear();
  // Load CSV to matrix of strings
  std::vector<std::vector<std::string> > temp = loadCSV(fileName);
  assert(temp.size() >= 1 && "CSV file is empty.");
  assert(temp.at(0).size() >= 3 && "CSV does not have the expected number of fields (atleast 2 times and 1 value).");
  // Iterate over the rows of the CSV and use comma-separated fields to popular outputs
  const unsigned vOffset = 2;
  const unsigned vDim = temp.at(0).size() - vOffset;
  Eigen::VectorXd tempVec(vDim);
  for (const auto& it : temp) {
    outTimes0->push_back((curves::Time)atof(it.at(1).c_str()));
    outTimes1->push_back((curves::Time)atof(it.at(2).c_str()));
    for (unsigned vIdx = 0; vIdx < vDim; vIdx++) {
      tempVec[vIdx] = atof(it.at(vIdx + vOffset).c_str());
    }
    outValues->push_back(tempVec);
  }
}

}  // namespace curves