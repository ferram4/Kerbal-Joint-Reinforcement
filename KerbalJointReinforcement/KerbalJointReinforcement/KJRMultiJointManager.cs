/*
Kerbal Joint Reinforcement, v3.1.2
Copyright 2015, Michael Ferrara, aka Ferram4

    This file is part of Kerbal Joint Reinforcement.

    Kerbal Joint Reinforcement is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Kerbal Joint Reinforcement is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Kerbal Joint Reinforcement.  If not, see <http://www.gnu.org/licenses/>.
*/
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace KerbalJointReinforcement
{
    //All this class exists to do is to act as a box attached to 
    //For a sequence of three parts (A, B, C), connected in series, this will exist on B and hold the strengthening joint from A to C
    //If the joint from A to B or B to C is broken, this will destroy the joint A to C and then destroy itself
    class KJRMultiJointManager
    {
        public static KJRMultiJointManager fetch;

        Dictionary<Part, List<ConfigurableJoint>> multiJointDict;

        public KJRMultiJointManager()
        {
            multiJointDict = new Dictionary<Part, List<ConfigurableJoint>>();
            fetch = this;
            GameEvents.onVesselCreate.Add(VesselCreate);
            GameEvents.onPartUndock.Add(OnJointBreak);
            GameEvents.onPartDie.Add(OnJointBreak);
        }

        public void OnDestroy()
        {
            Debug.Log("KJRMultiJointManager cleanup");
            GameEvents.onVesselCreate.Remove(VesselCreate);
            GameEvents.onPartUndock.Remove(OnJointBreak);
            GameEvents.onPartDie.Remove(OnJointBreak);
        }
        
        //This entire scheme relies on a simple fact: when a vessel is created, the part that was decoupled is the root part
        //Therefore, we only need to use vessel.RootPart
        private void VesselCreate(Vessel v)
        {
            //Debug.Log(v.name + " joint break");
            OnJointBreak(v.rootPart);
        }

        public void RegisterMultiJoint(Part testPart, ConfigurableJoint multiJoint)
        {
            List<ConfigurableJoint> configJointList;
            if (multiJointDict.TryGetValue(testPart, out configJointList))
            {
                configJointList.Add(multiJoint);
            }
            else
            {
                configJointList = new List<ConfigurableJoint>();
                configJointList.Add(multiJoint);
                multiJointDict.Add(testPart, configJointList);
            }
        }

        public void OnJointBreak(PartJoint partJoint)
        {
            OnJointBreak(partJoint.Parent);
        }

        public void OnJointBreak(Part part)
        {
            if (part == null)
                return;
            List<ConfigurableJoint> configJointList;
            if (multiJointDict.TryGetValue(part, out configJointList))
            {
                for(int i = 0; i < configJointList.Count; i++)
                {
                    ConfigurableJoint joint = configJointList[i];
                    if(joint != null)
                        GameObject.Destroy(joint);
                }

                multiJointDict.Remove(part);
            }
        }
    }
}
