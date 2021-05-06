#ifndef TEMOTO_RESOURCE_REGISTRAR__RR_QUERY_CONTAINER_H
#define TEMOTO_RESOURCE_REGISTRAR__RR_QUERY_CONTAINER_H

namespace temoto_resource_registrar
{
  template <class RawData>
  class QueryContainer
  {
  public:
    QueryContainer() : empty_(true){};
    QueryContainer(RrQueryBase q,
                   RawData req,
                   RawData data,
                   const std::string &server) : q_(q),
                                                raw_request_(req),
                                                raw_query_(data),
                                                responsible_server_(server),
                                                empty_(false)
    {
      storeNewId(q.id(), q.origin());
    };

    void storeNewId(const std::string &id, const std::string &rr)
    {
      rr_ids_[id] = rr;
    }

    void removeId(const std::string &id)
    {
      rr_ids_.erase(id);
    }

    int getIdCount() { return rr_ids_.size(); }

    RawData raw_query_;
    RawData raw_request_;
    RrQueryBase q_;
    std::string responsible_server_;

    std::unordered_map<std::string, std::string> rr_ids_;

    bool empty_;

  protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* version */)
    {
      ar &q_ &raw_request_ &raw_query_ &rr_ids_ &responsible_server_ &empty_;
    }
  };
} // namespace temoto_resource_registrar
#endif