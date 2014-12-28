/*
Kerbal Joint Reinforcement, v3.0
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
        class TwoConfigJointTuple
        {
            public ConfigurableJoint A, B;
        }

        Dictionary<PartJoint, TwoConfigJointTuple> multiJointDict;

        public KJRMultiJointManager()
        {
            multiJointDict = new Dictionary<PartJoint, TwoConfigJointTuple>();
            GameEvents.onPartJointBreak.Add(OnJointBreak);
        }

        public void OnDestroy()
        {
            Debug.Log("KJRMultiJointManager cleanup");
            GameEvents.onPartJointBreak.Remove(OnJointBreak);
        }

        public void RegisterMultiJoint(PartJoint partJoint, ConfigurableJoint multiJoint)
        {
            TwoConfigJointTuple configTuple;
            if(multiJointDict.TryGetValue(partJoint, out configTuple))
            {
                configTuple.B = multiJoint;
            }
            else
            {
                configTuple = new TwoConfigJointTuple();
                configTuple.A = multiJoint;
                multiJointDict.Add(partJoint, configTuple);
            }
        }

        private void OnJointBreak(PartJoint partJoint)
        {
            if (partJoint == null)
                return;
            TwoConfigJointTuple configTuple;
            if (multiJointDict.TryGetValue(partJoint, out configTuple))
            {
                if (configTuple.A != null)
                    GameObject.Destroy(configTuple.A);

                if (configTuple.B != null)
                    GameObject.Destroy(configTuple.B);
                multiJointDict.Remove(partJoint);
            }
        }
    }
}
