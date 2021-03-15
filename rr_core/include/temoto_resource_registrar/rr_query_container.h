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
                                                rawRequest_(req),
                                                rawQuery_(data),
                                                responsibleServer_(server),
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

    RawData rawQuery_;
    RawData rawRequest_;
    RrQueryBase q_;
    std::string responsibleServer_;

    std::unordered_map<std::string, std::string> rr_ids_;

    bool empty_;

  protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int /* version */)
    {
      ar &q_ &rawRequest_ &rawQuery_ &rr_ids_ &responsibleServer_;
    }
  };
} // namespace temoto_resource_registrar
#endif