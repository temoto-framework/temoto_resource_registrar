#ifndef TEMOTO_LOGGING__STRING_FORMATTING_H
#define TEMOTO_LOGGING__STRING_FORMATTING_H

/*
 * Credit goes to https://stackoverflow.com/a/23742517
 */

#include <boost/format.hpp>
#include <string>

namespace temoto_logging
{
namespace details
{
  inline boost::format& formatImpl(boost::format& f)
  {
    return f;
  }

  template <typename Head, typename... Tail>
  boost::format& formatImpl(boost::format& f
  , Head const& head
  , Tail&&... tail)
  {
    return formatImpl(f % head, std::forward<Tail>(tail)...);
  }
}

template <typename... Args>
std::string format(std::string formatString, Args&&... args)
{
  boost::format f(std::move(formatString));
  return details::formatImpl(f, std::forward<Args>(args)...).str();
}
}
#endif