#include "short_localisation/csv_helper.h"

void CsvHelper::writeFile(const std::string &file_path)
{
  using namespace std;

  ofstream file_out;
  // ios::trunc clears the file if it exists before writing
  file_out.open(file_path, ios::trunc);

  for (string row : csv_data_)
  {
    file_out << row << "\n";
  }

  file_out.close();
}

void CsvHelper::setNextRow(const std::vector<std::string> &row_data,
                           const std::string &delim)
{
  using namespace std;

  string row{""};
  for (string cell : row_data)
  {
    row.append(cell);
    row.append(delim);
  }

  csv_data_.push_back(row);
}
