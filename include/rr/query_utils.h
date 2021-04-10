#include <type_traits>

namespace QueryUtils
{
  struct No
  {
  };

  template <typename T, typename Arg>
  No operator==(const T &, const Arg &);

  template <typename T, typename Arg = T>
  struct EqualExists
  {
    enum
    {
      value = !std::is_same<decltype(*(T *)(0) == *(Arg *)(0)), No>::value
    };
  };
}