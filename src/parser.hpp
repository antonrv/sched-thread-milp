#ifndef STM_PARSER_HPP
#define STM_PARSER_HPP

#include <map>
#include <string>
#include <vector>

namespace stm::parser {

using arg_dict_t = std::map<std::string, std::string>;

arg_dict_t build_map(int argc, char **argv);

std::vector<std::string> split(std::string input, const char div);

} // namespace stm::parser

#endif // STM_PARSER_HPP
