/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef CHARGER_H
#define CHARGER_H
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/node.h"
#include "ns3/vector.h"
#include "ns3/timer.h"
#include "ns3/wsngr.h"
#include "ns3/singleton.h"
#include <vector>
#include <set>
#include<iostream>
#include<unordered_set>
#include<unordered_map>
namespace ns3 {
namespace charger{

constexpr double ALPHA = 0.01;

struct ChargerBase{

    enum State{
        IDLE,CHARGING,SELF_CHARGING,MOVING
    };
    constexpr static double CHARGING_RATE = 10;// J/s
    constexpr static double MOVING_RATE = 5;// m/s
    constexpr static double MOVING_ENERGY =45;// J/m
    constexpr static double MAX_ENERGY = 210000;// J
    static Time SELF_CHARGING_TIME;
    static Time FIRST_REQUEST_TIME;


    Vector position;
    double energy = MAX_ENERGY;

    State state{IDLE};

    Ipv4Address working_node;
    Timer checkTimer;
    Timer eventTimer;

    static int return_sink;
    static int dead_node;
    static double energy_in_moving;
    static double energy_for_nodes;
    static int charged_sensor;
    static int requested_sensor;
    static double tot_latency;
    // static Time first_dead;
    static Time first_request_time;

    struct ChargerCmp{
        bool operator()(const Ipv4Address& a,const Ipv4Address& b)const{
            if(a == b) return false;
            auto& x = wsngr::RoutingProtocol::nodes[a];
            auto& y = wsngr::RoutingProtocol::nodes[b];
            auto charger = Singleton<ChargerBase>::Get();

            return CalculateDistance(x.position,charger->position) < CalculateDistance(y.position,charger->position);
        }
    };

    struct IpHash{
        size_t operator()(const Ipv4Address& ip)const{
            return std::hash<uint32_t>{}(ip.Get());
        }
    };

    std::unordered_set<Ipv4Address,IpHash> chargeQueue;

    ChargerBase();

    double getChargingEfficiency(double distance);
    Time getMovingTime(double distance);
    Time getChargingTime(double energy);
    void handle();
    void checkHandle();
    void run();

    void print_statistics();
};
//?????????????????????
void Maximum_Independent_Sets();
bool check_MIS(std::vector<Ipv4Address>MIS,Ipv4Address value);


// ChargerBase& getChargeBase();


}
}

#endif /* CHARGER_H */