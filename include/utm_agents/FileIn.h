// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_FILEIO_INCLUDE_FILEIN_H_
#define SRC_FILEIO_INCLUDE_FILEIN_H_

#ifndef _WIN32
#include <sys/stat.h>
#else
#include <direct.h>
#endif  // !WIN32
#include <map>
#include <string.h>
#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>

#define STRING_UNINITIALIZED "string_uninitialized"

typedef std::map<std::string, double> VarMap;

template<typename T>
using vector2 = std::vector<std::vector<T> >;

template<typename T>
using vector3 = std::vector<vector2<T> >;
using std::string;
using std::istringstream;
using std::vector;
using std::ifstream;
using std::cin;
using std::stod;

/*!
A namespace that handles variable setting and standard file reading operations.
*/

namespace cio {

  enum ErrorCodes { UNRECOGNIZED_EXTENSION, NOT_PAIR };

  void variableNotFound(std::string var_name);
  void failedFile(std::string file_name);
  void unrecognizedExtension(std::string extension_name);
  void notPair(std::string file_name);
  bool getYesNo(std::string question);
  bool fileExists(std::string filename);
  VarMap readVariableFile(std::string file_name);
  std::vector<std::string> divide(std::string my_string,
  const std::string &separator);
  std::string detectSeparator(std::string file_name);
  void stripFilePath(std::string *file_name);
  void stripExtension(std::string *file_name);
  void mkdir_p(std::string pathname);

  template <typename DataType, size_t N>
  static void print_array(DataType data[], std::string label = "") {
    printf("Showing %s:\n\n", label.c_str());
    for (int i = 0; i < N; i++) {
      std::cout << data[i] << ", ";
    }
    printf("\n\n");
  }

  //! Takes in address of first element of 2D array and prints out
  template <typename DataType, size_t N, size_t M>
  static void print_array(DataType array_first[N][M], std::string label = "") {
    printf("Showing %s:\n\n", label.c_str());
    for (int i = 0; i < N; i++) {
      for (int j = 0; j < M; j++) {
        std::cout << array_first[i][j] << ", ";
      }
      printf("\n");
    }
    printf("\n\n");
  }

  //! Helper function for 1- and 2-d file output
  template <class T>
  void print(std::vector<T> &output, std::ofstream &file, std::string separator) {
    for (T outer : output) {
      file << outer << separator;
    }
  }

  //! Output function for 1d vector, list, etc: anything that can be iterated over
  template<class T>
  void print(std::vector<T> &output, std::string file_name,
    bool overwrite = true, std::string separator = ",") {
    std::ofstream file;
    if (overwrite) {
      file.open(file_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    } else {
      file.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);
    }

    if (!overwrite) file << '\n';

    if (file.is_open()) {
      print(output, file, separator);
      file.close();
      printf("... Successfully wrote to file %s.\n", file_name.c_str());
    } else {
      failedFile(file_name);
    }
  }


  //! Output function for 2d vector, list, etc: anything that can be iterated over
  template<class T>
  void print2(const vector2<T> &output, std::string file_name,
  bool overwrite = true, std::string separator = ",") {
    std::ofstream file;
    if (overwrite) {
      file.open(file_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    } else {
      file.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);
    }

    if (!overwrite) file << '\n';

    if (file.is_open()) {
      for (std::vector<T> outer : output) {
        print(outer, file, separator);
        file << "\n";
      }
      file.close();
      printf("... Successfully wrote to file %s.\n", file_name.c_str());
    } else {
      failedFile(file_name);
    }
  }

  //! Helper function for 3-d file output
  template <class T>
  void print2(const vector2<T> &output, std::ofstream &file,
    std::string separator) {
    for (std::vector<T> outer : output) {
      print(outer, file, separator);
      file << "\n";
    }
  }
  
  //! Output function for 3d vector, list, etc: anything that can be iterated over
  template <class T>
  void print3(const vector3<T> &output, std::string file_name,
    bool overwrite = true, std::string separator = ",") {
    std::ofstream file;
    if (overwrite) {
      file.open(file_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    } else {
      file.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);
      file << '\n';
    }

    if (file.is_open()) {
      for (auto outer : output) {
        cio::print2(outer, file, separator);
        file << "\n";
      }
      file.close();
      printf("... Successfully wrote to file %s.\n", file_name.c_str());
    } else {
      failedFile(file_name);
    }
  }

  //! Reads in a 2D data file
  template <typename T = double>
  vector2<T> read2(std::string file_name,
  std::string separator = STRING_UNINITIALIZED) {
    if (separator == STRING_UNINITIALIZED)
      separator = detectSeparator(file_name);

    std::ifstream file(file_name.c_str());
    if (!file.is_open()) {
      failedFile(file_name);
      return vector2<T>();
    }

    std::string value;
    vector2<T> file_matrix;

    while (file.good()) {
      getline(file, value);
      std::istringstream iss(value);
      std::string word;
      std::vector<T> line;

      std::stringstream conv;
      T val;

      while (getline(iss, word, *separator.c_str())) {
        conv << word;
        conv >> val;
        line.push_back(val);
        conv.clear();
      }
      if (line.size()) {
        file_matrix.push_back(line);
      }
    }
    file.close();

    printf("... Successfully read in file %s.\n", file_name.c_str());
    return file_matrix;
  }

  /* Takes an input dictating size of 2nd dimension */
  template <class T>
  vector3<T> read3(std::string file_name, size_t dim2, bool overwrite = true, std::string separator = ",") {
    vector2<T> v = read2<T>(file_name);

    vector3<T> vout(v.size()/dim2);
    for (size_t i = 0; i < v.size(); i++) {
      vout[static_cast<int>(static_cast<double>(i) / dim2)].push_back(v[i]);
    }
    return vout;
  }

  //! Print-to-file for a 1D container of pairs (also works with maps).
  template<class T>
  void printPairs(T data, std::string file_name, bool overwrite = true, std::string separator = ",") {
    std::ofstream file;
    if (overwrite) {
      file.open(file_name.c_str(), std::ofstream::out | std::ofstream::trunc);
    } else {
      file.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);
    }

    if (!overwrite) file << '\n';

    if (file.is_open()) {
      for (auto it : data) {
        file << it.first << separator << it.second << separator << "\n";
      }
      file.close();
      printf("... Successfully wrote to file %s.\n", file_name.c_str());
    } else {
      failedFile(file_name);
    }
  }


  //! Converts an element from one type to another using stringstream.
  template <typename T1, typename T2>
  T1 convert(const T2 &input) {
    std::stringstream convert;
    convert << input;
    T1 output;
    convert >> output;
    return output;
  }

  /*!
  Converts a vector to a vector of another type using stringstream.
  Requires call of type convert<T1>.
  */
  template <typename T1, typename T2>
  vector2<T1> convert(const vector2<T2> &f) {
    std::vector<std::vector<T1> > var(f.size());
    for (unsigned int i = 0; i < f.size(); i++) {
      var[i] = std::vector<T1>(f[i].size());
      for (unsigned int j = 0; j < f[i].size(); j++) {
        var[i][j] = convert<T1>(f[i][j]);
      }
    }
    return var;
  }

  //! Read in pair type objects
  template<typename T>
  std::vector<T> readPairs(std::string file_name, std::string separator = STRING_UNINITIALIZED) {
    std::vector<std::vector<double> >  m = read2<double>(file_name, separator);
    std::vector<T> var;
    for (std::vector<double> i : m) {
      if (i.size() != 2) {
        throw NOT_PAIR;
      } else {
        var.push_back(T(i[0], i[1]));
      }
    }
    return var;
  }
  
  void mkdir_p(string pathname) {
    char *path = (char*)pathname.c_str();
    char *token = NULL;
    char *context = NULL;
    char delims[] = "/";

  #ifdef _WIN32
    token = strtok_s(path, delims, &context);
  #else
    token = strtok_r(path, delims, &context);
  #endif

    string tmp = "";

    // keep going until there are no tokens left
    while (token != NULL) {
      // append the next token to the path
      tmp += token;

      // Create the directory
  #ifdef _WIN32
      _mkdir(tmp.c_str());
  #else
      mkdir(tmp.c_str(),0775);
  #endif

      tmp += "/";  // append a / to the path for the next token and get it

  #ifdef _WIN32
      token = strtok_s(NULL, delims, &context);
  #else
      token = strtok_r(NULL, delims, &context);
  #endif
    }
  //    delete[]token;
  //    delete[]path;
  //    delete[]context;
  //    delete[]&delims;
  }

  void stripFilePath(string *file_name) {
    istringstream iss(*file_name);
    string token;
    // Returns the last element in the filepath
    while (getline(iss, token, '/')) {}
    *file_name = token;
  }

  void stripExtension(string *file_name) {
    size_t found;
    found = file_name->find('.');
    file_name->resize(found);
  }

  //! Divides a string by 'separator'
  vector<string> divide(string my_string, const string &separator) {
    vector<string> divided;
    while (my_string.find(separator) != string::npos) {
      // Find the separator
      size_t found = my_string.find(separator);
      // Cut at separator
      divided.push_back(my_string.substr(0, found));
      // Remove to separator
      my_string.erase(my_string.begin(), my_string.begin() + found + 1);
    }
    divided.push_back(my_string);
    return divided;
  }

  bool getYesNo(string question) {
    while (true) {
      string user_entry;
      printf("%s (y/n)\t", question.c_str());
      cin >> user_entry;
      if (user_entry == "y") {
        return true;
      } else if (user_entry == "n") {
        return false;
      } else {
        printf("Unrecognized entry. ");
      }
    }
  }

  string detectSeparator(string file_name) {
    vector<string> divided = divide(file_name, ".");
    if (divided.back() == "csv") {
      return ",";
    } else if (divided.back() == "xls") {
      return "\t";
    } else {
      unrecognizedExtension(divided.back());
      return "";
    }
  }

  bool fileExists(string filename) {
    ifstream myfile(filename);
    bool exists = myfile.good();
    myfile.close();
    return exists;
  }

  //! Reads variables (double) from a file to access by name (string)
  VarMap readVariableFile(string file_name) {
    vector2<string> raw_data = read2<string>(file_name);
    VarMap  processed_data;
    for (vector<string> &r : raw_data) {
      auto p = make_pair(r[0], stod(r[1].c_str()));
      processed_data.insert(p);
    }
    return processed_data;
  }


  void variableNotFound(string var_name) {
    printf("Variable %s not found!\n", var_name.c_str());
    system("pause");
  }

  //! Error that prints when a file was not opened.
  void failedFile(string file_name) {
    printf("Failed to open %s.\n", file_name.c_str());
  }

  //! Error that prints, pauses, and closes the program.
  void unrecognizedExtension(string extension_name) {
    printf("Unknown extension %s: ", extension_name.c_str());
    printf("please specify separator type. Aborting.\n");
    system("pause");
  }

  //! Error that prints, pauses, and closes the program when pair detection fails.
  void notPair(string file_name) {
    printf("Error! %s ", file_name.c_str());
    printf("does not contain pair values. Aborting.\n");
    system("pause");
  }

}  // namespace cio
#endif  // SRC_FILEIO_INCLUDE_FILEIN_H_
