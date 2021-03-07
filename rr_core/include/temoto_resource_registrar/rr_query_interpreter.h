#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_QUERY_INTER_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_QUERY_INTER_H

namespace temoto_resource_registrar
{
  template <class QueryClass>
  class RrQueryInterpreter
  {
  public:
    virtual void rr(QueryClass &query, const std::string rrName) = 0;
    virtual std::string rr() = 0;

    virtual void origin(QueryClass &query, const std::string rrName) = 0;
    virtual std::string origin() = 0;

    virtual void id(QueryClass &query, const std::string id) = 0;
    virtual std::string id() = 0;
  protected:
  private:
  };
}
#endif