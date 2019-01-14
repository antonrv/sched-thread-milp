#include "parser.hpp"

#include <sstream>
#include <vector>

namespace stm::parser {

// in case of error
static std::string string_cat(const std::vector<std::string> &vStr) {
  std::string ret;
  for (auto &s : vStr) {
    ret += s + ' ';
  }
  return ret;
}

static bool check_opt_format(const std::string str) { return str[0] == '-'; }

std::vector<std::string> split(std::string input, const char div) {
  std::vector<std::string> ret;
  std::istringstream iss(input);
  std::string s;
  while (std::getline(iss, s, div)) {
    if (not s.empty()) {
      ret.push_back(s);
    }
  }
  return ret;
}

arg_dict_t build_map(int argc, char **argv) {
  std::vector<std::string> vArgs;
  bool trimBinaryName = true;
  if (argc == 2) {
    // add_test is maybe messing with the args
    std::string argSet = argv[1];
    vArgs = split(argSet, ' ');
    trimBinaryName = false;
  } else {
    vArgs = std::vector<std::string>(argv, argv + argc);
    trimBinaryName = true;
  }

  std::vector<std::string> vExtArgs;

  std::vector<std::string> keys;
  std::vector<std::string> values;
  arg_dict_t parsedMap;

  for (auto t : vArgs) {
    auto tokens = split(t, '=');
    vExtArgs.insert(std::end(vExtArgs), std::begin(tokens), std::end(tokens));
  }

  std::string binName = "bin-name";
  if (trimBinaryName) {
    binName = vExtArgs.front();
    vExtArgs.erase(std::begin(vExtArgs));
  }

  auto optCount = vExtArgs.size();
  if (optCount % 2 != 0) {
    // we require key - value pairs
    throw std::runtime_error("Unexpected number of arguments (" +
                             std::to_string(optCount) +
                             ")in : " + string_cat(vArgs));
  }
  for (auto i = 0; i < optCount; i += 2) {
    auto keyStr = vExtArgs[i];
    auto valStr = vExtArgs[i + 1];
    auto correctKey = check_opt_format(keyStr);
    auto correctValue = not check_opt_format(valStr);
    if (not correctKey) {
      throw std::runtime_error("Incorrect option format in : " + keyStr +
                               ". Allowed : [--my-option=value] / [--my-option "
                               "value] / [-o value]\n" +
                               "Original: " + string_cat(vArgs));
    }
    if (not correctValue) {
      throw std::runtime_error(
          "Incorrect value format in : " + valStr +
          "[--my-option=value] / [--my-option value] / [-o value]\n" +
          "Original: " + string_cat(vArgs));
    }
    if (parsedMap.find(keyStr) != std::end(parsedMap)) {
      throw std::runtime_error("Duplicated option : " + keyStr +
                               " in : " + string_cat(vArgs));
    }
    parsedMap[keyStr] = valStr;
  }
  return parsedMap;
}

} // namespace stm::parser
