#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

class CsvHelper
{
public:
  void writeFile(const std::string &file_path);
  void setNextRow(const std::vector<std::string> &row_data,
                  const std::string &delim);

private:
  std::vector<std::string> csv_data_; ///< data for reading or writing to csv
};
