#include <iostream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>

void _delimeter_tokenizer(std::string& data, std::vector<std::string>& tokens, std::string delimiter)
{
  boost::split(tokens, data, boost::is_any_of(delimiter));
}

typedef boost::function<void(std::string&, std::vector<std::string>&)> TokenizerType;

int main(void)
{
  std::string data = "a\rb\rc\r";
  std::vector<std::string> tokens;
  std::string delimiter = "\r";

  TokenizerType f = boost::bind(_delimeter_tokenizer, _1, _2, delimiter);
  f(data, tokens);

  BOOST_FOREACH (std::string token, tokens)
    std::cout << token << std::endl;

  return 0;
}
