/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "charger.h"

namespace ns3 {
namespace charger{


Time ChargerBase::SELF_CHARGING_TIME = Seconds(1800);
Time ChargerBase::FIRST_REQUEST_TIME=Seconds(7200);

int ChargerBase::dead_node = 0;
double ChargerBase::energy_in_moving = 0;
double ChargerBase::energy_for_nodes = 0;
int ChargerBase::charged_sensor = 0;
int ChargerBase::requested_sensor = 0;
double ChargerBase::tot_latency = 0;

ChargerBase::ChargerBase():
    eventTimer(Timer::CANCEL_ON_DESTROY),
    checkTimer(Timer::CANCEL_ON_DESTROY)
{
    auto& sink = wsngr::RoutingProtocol::getSink();
    position = sink.position;

}


double ChargerBase::getChargingEfficiency(double distance){
    return 0.68;
}
void ChargerBase::handle(){
    std::cout << "state : " << state << "\n";
    switch(state){
        case IDLE:{
            if(!chargeQueue.size()) return;

            // charge self
            auto& sink = wsngr::RoutingProtocol::getSink();

            auto ip = *chargeQueue.begin();

            auto& node = wsngr::RoutingProtocol::nodes[ip];
            double dist = CalculateDistance(node.position,position);

            double tot_dist = dist + CalculateDistance(node.position,sink.position);

            double moveing_to_sink_energy = tot_dist * MOVING_ENERGY + (wsngr::NodeInfo::MAX_ENERGY - node.energy) / getChargingEfficiency(0);
            std::cout << " " << moveing_to_sink_energy << " ip " << ip << "\n";
            if(energy < moveing_to_sink_energy){
                state = SELF_CHARGING;
                Time time = getMovingTime(CalculateDistance(position,sink.position)) + SELF_CHARGING_TIME;
                // === record
                energy_in_moving += CalculateDistance(position,sink.position) * MOVING_ENERGY;

                // ===
                energy = MAX_ENERGY;
                eventTimer.Schedule(time);
                return;
            }
            Time time = getMovingTime(dist);
            // === record
            energy_in_moving += dist * MOVING_ENERGY;
            // ===
            state = MOVING;
            working_node = ip;
            eventTimer.Schedule(time);
        }
        break;
        case CHARGING:{
            state = IDLE;
            auto& node = wsngr::RoutingProtocol::nodes[working_node];
            chargeQueue.erase(working_node);
            node.state = wsngr::NodeState::WORKING;
            node.energy_consume_in_record_intervel = 0;
            node.last_update_time = Simulator::Now();
            // === record
            tot_latency += (Simulator::Now() - node.requested_time).GetSeconds();
            // ===
            eventTimer.Schedule(Seconds(0.1));
        }
        break;
        case SELF_CHARGING:{
            state = IDLE;
            auto& sink = wsngr::RoutingProtocol::getSink();
            position = sink.position;
        }
        break;
        case MOVING:{
            state = CHARGING;
            auto& node = wsngr::RoutingProtocol::nodes[working_node];
            position = node.position;
            Time time = getChargingTime(node.energy);

            energy -= (wsngr::NodeInfo::MAX_ENERGY - node.energy) / getChargingEfficiency(0);

            // === record
            energy_for_nodes += wsngr::NodeInfo::MAX_ENERGY - node.energy;
            charged_sensor++;
            // ===

            node.state = wsngr::NodeState::CHARGING;
            node.energy = wsngr::NodeInfo::MAX_ENERGY;

            eventTimer.Schedule(time);
        }
        break;
    }
}

Time ChargerBase::getMovingTime(double distance){
    return Seconds(distance / MOVING_RATE);
}

Time ChargerBase::getChargingTime(double energy){
    return Seconds((wsngr::NodeInfo::MAX_ENERGY - energy) / CHARGING_RATE * getChargingEfficiency(0));
}

void ChargerBase::checkHandle(){

    auto& nodes = wsngr::RoutingProtocol::GetNodes();
    for(auto& [ip,info] : nodes){
        if(info.energy <= wsngr::NodeInfo::CHARING_THRESHOLD){
            if(!chargeQueue.count(ip)){
                info.requested_time = Simulator::Now();
                if(FIRST_REQUEST_TIME==Seconds(7200)){
                    FIRST_REQUEST_TIME=info.requested_time;
                }
                requested_sensor++;
                chargeQueue.insert(ip);
            }
        }
    }
    if(chargeQueue.size() && state == IDLE && eventTimer.IsExpired()){
        eventTimer.Schedule(Seconds(0.1));
    }
    checkTimer.Schedule(Seconds(20));
}


void ChargerBase::run(){

    eventTimer.SetFunction(&ChargerBase::handle,this);
    checkTimer.SetFunction(&ChargerBase::checkHandle,this);

    checkTimer.Schedule(Seconds(20));
    eventTimer.Schedule(Seconds(5));
}

void ChargerBase::print_statistics(){

    auto& nodes = wsngr::RoutingProtocol::GetNodes();
    Time first_dead_time=Seconds(7200);
    for(auto& [ip,info] : nodes){
        dead_node += info.deadTimes;
        if(info.first_dead_time<=first_dead_time){
            first_dead_time=info.first_dead_time;
        }
    }
    std::cout << "======statistics======\n"
        << "dead nodes : " << dead_node << " \n"
        << "first dead time : " << first_dead_time.GetSeconds() << " \n"
        << "first request time : " << FIRST_REQUEST_TIME.GetSeconds() << " \n"
        << "energy in moving : " << energy_in_moving << " \n"
        << "nodes receive energy : " << energy_for_nodes << "\n"
        << "energy usage efficiency: " << energy_for_nodes / (energy_for_nodes / getChargingEfficiency(0) + energy_in_moving) << "\n"
        << "survival rate: " << charged_sensor*1.0 / requested_sensor << "\n"
        << "average charging latency: " << tot_latency / charged_sensor << "s\n"
        << "======================";
}

bool check_MIS(std::vector<Ipv4Address>MIS,Ipv4Address value){
    bool result=false;
    for(int i=0;i<MIS.size();i++){
        if(MIS[i]==value){
            result=true;
        }
    }
    return result;

}

void Maximum_Independent_Sets(){
    //std::cout<<" MIS1 "<<std::endl;
    auto& nodes = wsngr::RoutingProtocol::GetNodes();
    //std::cout<<" MIS2 "<<std::endl;
    std::vector<std::vector<Ipv4Address>>sort;
    std::vector<Ipv4Address> MIS,UNMIS;
    sort.resize(1000);
    int count1=0;
    int count2=0;
    int count3=0;
    int count4=0;
    int count5=0;
    int count6=0;
    for(auto& [ip,info] : nodes){
        std::vector<Ipv4Address> ne_vec = info.m_chagerstate.GetNeighbors();
        //std::cout<<" MIS3 "<<std::endl;
        if(ne_vec.size()==0)count1++;
        sort[ne_vec.size()].push_back(ip);
        //std::cout<<" MIS4 "<<std::endl;
    }
    //std::cout<<" MIS3 "<<std::endl;
    for(int i=0;i<sort.size();i++){
        if(sort[i].size()==0){
            count2++;
            continue;
        }
        if(i==0){
            for(int m=0;m<sort[i].size();m++){
                if(check_MIS(MIS,sort[i][m])==false&&check_MIS(UNMIS,sort[i][m])==false){
                    MIS.push_back(sort[i][m]);
                    count3++;
                }
            }
            continue;
        }
        //std::cout<<i<<":"<<sort[i].size()<<std::endl;
        else{
            for(int j=0;j<sort[i].size();j++){
                if(check_MIS(UNMIS,sort[i][j])==true||check_MIS(MIS,sort[i][j])==true){
                    count6++;
                    continue;
                }
                else{
                    MIS.push_back(sort[i][j]);
                    count4++;
                    std::vector<Ipv4Address> ne_vec=nodes[sort[i][j]].m_chagerstate.GetNeighbors();
                    std::cout<<"size"<<ne_vec.size()<<std::endl;
                    for(int k=0;k<ne_vec.size();k++){
                         std::cout<<k<<" ne_vec "<<ne_vec[k]<<std::endl;
                        if(check_MIS(UNMIS,ne_vec[k])==false&&check_MIS(MIS,ne_vec[k])==false){
                            UNMIS.push_back(ne_vec[k]);
                            count5++;
                        }
                    }
                }   
            }

        }
        
    }
    std::cout<<" count1 "<<count1<<" count2 "<<count2<<" count3 "<<count3<<" count4 "<<count4<<" count5 "<<count5<<" count6 "<<count6<<" MIS "<<MIS.size()<<" UNMIS " <<UNMIS.size()<<std::endl;
    // for(int s=0;s<MIS.size();s++){
    //     std::cout<<nodes[MIS[s]].id<<std::endl;
    // }
}



}
}

