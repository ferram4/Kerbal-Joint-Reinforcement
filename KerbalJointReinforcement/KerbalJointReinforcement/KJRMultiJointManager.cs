/*
Kerbal Joint Reinforcement, v3.3.3
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
		Dictionary<Part, List<ConfigurableJoint>> multiJointDict;
		List<Part> linkedSet;
		List<Part> tempPartList;

		public KJRMultiJointManager()
		{
			multiJointDict = new Dictionary<Part, List<ConfigurableJoint>>();
			linkedSet = new List<Part>();
			tempPartList = new List<Part>();

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
		//Therefore, we only need to use vessel.RootPart and the parts with no children to ensure that all multijoints are broken
		private void VesselCreate(Vessel v)
		{
			//Debug.Log(v.name + " joint break");
			OnJointBreak(v.rootPart);

			for(int i = 0; i < v.Parts.Count; ++i)
			{
				Part p = v.Parts[i];
				OnJointBreak(p);
			}
		}

		public bool TrySetValidLinkedSet(Part linkPart1, Part linkPart2)
		{
			linkedSet.Clear();
			tempPartList.Clear();

			while(linkPart1 != null)
			{
				linkedSet.Add(linkPart1);
				linkPart1 = KJRJointUtils.JointAdjustmentValid(linkPart1) ? linkPart1.parent : null;
			}

			while(linkPart2 != null)
			{
				tempPartList.Add(linkPart2);
				linkPart2 = KJRJointUtils.JointAdjustmentValid(linkPart2) ? linkPart2.parent : null;
			}

			int i = linkedSet.Count - 1;
			int j = tempPartList.Count - 1;

			if(linkedSet[i] != tempPartList[j])
				return false; // not same root, so they can never be in a valid set

			while((i >= 0) && (j >= 0) && (linkedSet[i] == tempPartList[j]))
			{ --i; --j; }

			linkedSet.AddRange(tempPartList.GetRange(0, j + 1)); 

			return linkedSet.Count > 1;
		}

		public void RegisterMultiJointBetweenParts(Part linkPart1, Part linkPart2, ConfigurableJoint multiJoint)
		{
			foreach(Part p in linkedSet)
				RegisterMultiJoint(p, multiJoint);
		}

		public void RegisterMultiJoint(Part testPart, ConfigurableJoint multiJoint)
		{
			List<ConfigurableJoint> configJointList;
			if(multiJointDict.TryGetValue(testPart, out configJointList))
			{
				for(int i = configJointList.Count - 1; i >= 0; --i)
					if(configJointList[i] == null)
						configJointList.RemoveAt(i);

				configJointList.Add(multiJoint);
			}
			else
			{
				configJointList = new List<ConfigurableJoint>();
				configJointList.Add(multiJoint);
				multiJointDict.Add(testPart, configJointList);
			}
		}

		public bool CheckMultiJointBetweenParts(Part testPart1, Part testPart2)
		{
			if(testPart1 == null || testPart2 == null || testPart1 == testPart2)
				return false;

			List<ConfigurableJoint> testMultiJoints;

			if(!multiJointDict.TryGetValue(testPart1, out testMultiJoints))
				return false;

			Rigidbody testRb = testPart2.rb;

			for(int i = 0; i < testMultiJoints.Count; i++)
			{
				ConfigurableJoint joint = testMultiJoints[i];
				if(joint == null)
				{
					testMultiJoints.RemoveAt(i);
					--i;
					continue;
				}
				if(joint.connectedBody == testRb)
					return true;
			}

			return false;
		}

		public void OnJointBreak(PartJoint partJoint)
		{
			OnJointBreak(partJoint.Parent);
		}

		public void OnJointBreak(Part part)
		{
			if(part == null)
				return;
			List<ConfigurableJoint> configJointList;
			if(multiJointDict.TryGetValue(part, out configJointList))
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
