#include <uxr/agent/client/ProxyClient.hpp>
#include <uxr/agent/participant/Participant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <cassert>
#include <cstdlib>
#include <iostream>
using namespace eprosima;
void check(int requested, int expected, bool xml) {
  dds::xrce::CLIENT_Representation client_rep;
  client_rep.client_key({1,2,3,4}); client_rep.session_id(0x81); client_rep.mtu(512);
  auto client=std::make_shared<uxr::ProxyClient>(client_rep, uxr::Middleware::Kind::FASTDDS);
  dds::xrce::OBJK_PARTICIPANT_Representation rep;
  rep.domain_id(requested);
  if(xml) rep.representation().xml_string_representation("<dds><participant><rtps><name>fleet_test</name></rtps></participant></dds>");
  else rep.representation().binary_representation({0,0});
  rep.endianness(static_cast<dds::xrce::Endianness>(1));
  auto participant=uxr::Participant::create({0,1},client,rep);
  assert(participant);
  auto factory=fastdds::dds::DomainParticipantFactory::get_instance();
  assert(factory->lookup_participant(expected));
  if(requested!=expected) assert(!factory->lookup_participant(requested));
  dds::xrce::ObjectVariant variant; variant.participant(rep); variant.endianness(static_cast<dds::xrce::Endianness>(1));
  assert(participant->matched(variant));
  participant.reset();
  assert(!factory->lookup_participant(expected));
  std::cout << "domain " << requested << " -> " << expected << (xml?" XML":" binary") << " PASS\n";
}
int main() {
  unsetenv("LEAP_AGENT_DOMAIN_ID"); check(83,83,false);
  setenv("LEAP_AGENT_DOMAIN_ID","81",1); check(0,81,false);
  setenv("LEAP_AGENT_DOMAIN_ID","82",1); check(0,82,true);
}
