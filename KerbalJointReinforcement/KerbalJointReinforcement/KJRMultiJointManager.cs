/*
Kerbal Joint Reinforcement, v3.0.1
Copyright 2014, Michael Ferrara, aka Ferram4

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
        class TwoConfigJointTuple
        {
            public ConfigurableJoint A, B;
        }

        Dictionary<Part, TwoConfigJointTuple> multiJointDict;

        public KJRMultiJointManager()
        {
            multiJointDict = new Dictionary<Part, TwoConfigJointTuple>();
            fetch = this;
            GameEvents.onVesselCreate.Add(VesselCreate);
        }

        public void OnDestroy()
        {
            Debug.Log("KJRMultiJointManager cleanup");
            GameEvents.onVesselCreate.Remove(VesselCreate);
        }

        public void RegisterMultiJoint(PartJoint partJoint, ConfigurableJoint multiJoint)
        {
            RegisterMultiJoint(partJoint.Parent, multiJoint);
        }
        
        //This entire scheme relies on a simple fact: when a vessel is created, the part that was decoupled is the root part
        //Therefore, we only need to use vessel.RootPart
        private void VesselCreate(Vessel v)
        {
            //Debug.Log(v.name + " joint break");
            OnJointBreak(v.rootPart);
        }

        public void RegisterMultiJoint(Part part, ConfigurableJoint multiJoint)
        {
            TwoConfigJointTuple configTuple;
            if (multiJointDict.TryGetValue(part, out configTuple))
            {
                configTuple.B = multiJoint;
            }
            else
            {
                configTuple = new TwoConfigJointTuple();
                configTuple.A = multiJoint;
                multiJointDict.Add(part, configTuple);
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
            TwoConfigJointTuple configTuple;
            if (multiJointDict.TryGetValue(part, out configTuple))
            {
                if (configTuple.A != null)
                {
                    GameObject.Destroy(configTuple.A);
                    //Debug.Log("Destroyed A");
                }

                if (configTuple.B != null)
                {
                    GameObject.Destroy(configTuple.B);
                    //Debug.Log("Destroyed B");
                }
                multiJointDict.Remove(part);
            }
        }
    }
}
