/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2004 Francisco J. Ros
 * Copyright (c) 2007 INESC Porto
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Francisco J. Ros  <fjrm@dif.um.es>
 *          Gustavo J. A. M. Carneiro <gjc@inescporto.pt>
 */


#ifndef WSNGR_STATE_H
#define WSNGR_STATE_H
#include"ns3/ipv4-address.h"
#include<vector>
namespace ns3 {
namespace wsngr {

/// \ingroup olsr
/// This class encapsulates all data structures needed for maintaining internal state of an OLSR node.
class WsngrState
{
  typedef std::vector<Ipv4Address> NeighborSet; //!< Neighbor Set type.
  //typedef std::vector<Ipv4Address> ChagerNeighborSet;
protected:
  NeighborSet m_neighborSet; 
  //ChagerNeighborSet m_chagerneighborSet;        

public:
  WsngrState ()
  {
  }

  const NeighborSet & GetNeighbors () const
  {
    return m_neighborSet;
  }
  /**
   * Gets the neighbor set.
   * \returns The neighbor set.
   */
  NeighborSet & GetNeighbors ()
  {
    return m_neighborSet;
  }
};

}
}  // namespace olsr,ns3

#endif /* OLSR_STATE_H */
