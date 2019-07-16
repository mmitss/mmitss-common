//*************************************************************************************************************
//
// © 2016-2019 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*************************************************************************************************************
#include <algorithm>
#include <bitset>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>

// asn1
#include <asn_application.h>
#include "MessageFrame.h"
// asn1j2735
#include "AsnJ2735Lib.h"

/// convert number of bits to number of bytes
auto numbits2numbytes = [](const ssize_t& bit_nums)->size_t
	{return((bit_nums <= 0) ? (0) : ((bit_nums + 7) >> 3));};

/// convert unsigned long to OCTET STRING with preallocated buffer
auto ul2octString = [](uint8_t* pbuf, int size, unsigned long value)->void
{ // OCTET STRING is in network byte order
	for (int i = 0; i < size; i++)
	{
		unsigned int shift_bits = (size - 1 - i) * 8;
		pbuf[i] = (uint8_t)((value >> shift_bits) & 0xFF);
	}
};

/// convert OCTET STRING to unsigned long
auto octString2ul = [](const uint8_t* pbuf, const size_t& size)->unsigned long
{ // OCTET STRING is in network byte order
	unsigned long ret = 0;
	for (size_t i = 0; i < size; i++)
		ret = (ret << 8) | pbuf[i];
	return(ret);
};

/// reverse bit-order of unsigned long
auto reverseul = [](const unsigned long& value, const int& num_bits)->unsigned long
{
	std::string str = std::bitset<32>(value).to_string().substr(32 - num_bits);
	std::reverse(str.begin(), str.end());
	return(std::bitset<32>(str).to_ulong());
};

/// convert BIT STRING to unsigned long
auto bitString2ul = [](const uint8_t* pbuf, const size_t& size, const int& bits_unused)->unsigned long
{
	unsigned long ret = octString2ul(pbuf, size) >> bits_unused;
	int num_bits = static_cast<int>(size * 8 - bits_unused);
	return(reverseul(ret, num_bits));
};

/// convert unsigned long to BIT STRING
auto ul2bitString = [](uint8_t** pbuf, size_t& num_bytes, int& bits_unused, const int& num_bits, const unsigned long& value)->bool
{
	int bytes = (num_bits / 8) + (((num_bits % 8) > 0) ? 1 : 0);
	if ((*pbuf = (uint8_t *)calloc(bytes, sizeof(uint8_t))) == NULL)
		return(false);
	num_bytes = bytes;
	bits_unused = bytes * 8 - num_bits;
	ul2octString(*pbuf, bytes, reverseul(value, num_bits) << bits_unused);
	return(true);
};

/// convert uint32_t vehicle ID to Temporary ID
auto id2temporaryId = [](uint8_t** pbuf, size_t& size, const unsigned long& value)->bool
{ // Temporary ID has 4 bytes
	if ((*pbuf = (uint8_t *)calloc(4, sizeof(uint8_t))) == NULL)
		return(false);
	size = 4;
	ul2octString(*pbuf, 4, value);
	return(true);
};

/// fill MapData_t structure which specifies speed limit at lane/node level
auto mapData2msgFrame_single = [](const MapData_element_t& mapDataIn, MapData_t& mapData)->bool
{	// MapData:
	// -- Required objects ------------------------------------ //
	//	msgIssueRevision
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	MinuteOfTheYear                     EXCL
	//	LayerType                           INCL
	//	LayerID                             EXCL
	//	IntersectionGeometryList            INCL
	//	RoadSegmentList                     EXCL
	//	DataParameters                      EXCL
	//	RestrictionClassList                EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	std::string prog_name = "mapData2msgFrame_single: ";
	std::string allocate_level{"MapData"};
	// msgIssueRevision
	mapData.msgIssueRevision	= mapDataIn.mapVersion;
	// LayerType
	if ((mapData.layerType = (LayerType_t *)calloc(1, sizeof(LayerType_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".LayerType" << std::endl;
		return(false);
	}
	*(mapData.layerType) = LayerType_intersectionData;
	// IntersectionGeometryList - one intersection per MapData
	allocate_level += ".IntersectionGeometryList.IntersectionGeometry";
	if (((mapData.intersections = (IntersectionGeometryList_t *)calloc(1, sizeof(IntersectionGeometryList_t))) == NULL)
		|| ((mapData.intersections->list.array = (IntersectionGeometry_t **)calloc(1, sizeof(IntersectionGeometry_t *))) == NULL)
		|| ((mapData.intersections->list.array[0] = (IntersectionGeometry_t *)calloc(1, sizeof(IntersectionGeometry_t))) == NULL))
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << std::endl;
		return(false);
	}
	mapData.intersections->list.size = 1;
	mapData.intersections->list.count = 1;
	IntersectionGeometry_t* pIntersectionGeometry = mapData.intersections->list.array[0];
	// IntersectionGeometry:
	// -- Required objects ------------------------------------ //
	//	IntersectionReferenceID
	//	MsgCount
	//	Position3D
	//	LaneList
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	DescriptiveName                     EXCL
	//	LaneWidth                           INCL
	//	SpeedLimitList                      EXCL
	//	PreemptPriorityList                 EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	// IntersectionReferenceID
	pIntersectionGeometry->id.id = mapDataIn.id;
	if ((pIntersectionGeometry->id.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".id.region" << std::endl;
		return(false);
	}
	*(pIntersectionGeometry->id.region) = mapDataIn.regionalId;
	// MsgCount
	pIntersectionGeometry->revision = 0;
	// Position3D
	pIntersectionGeometry->refPoint.lat = mapDataIn.geoRef.latitude;
	pIntersectionGeometry->refPoint.Long = mapDataIn.geoRef.longitude;
	if (mapDataIn.attributes.test(0))
	{ // include elevation data
		if ((pIntersectionGeometry->refPoint.elevation = (DSRC_Elevation_t *)calloc(1, sizeof(DSRC_Elevation_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".Position3D.Elevation" << std::endl;
			return(false);
		}
		*(pIntersectionGeometry->refPoint.elevation) = mapDataIn.geoRef.elevation;
	}
	// LaneWidth
	uint16_t refLaneWidth = mapDataIn.mpApproaches[0].mpLanes[0].width;
	if ((pIntersectionGeometry->laneWidth = (LaneWidth_t *)calloc(1, sizeof(LaneWidth_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".LaneWidth" << std::endl;
		return(false);
	}
	*(pIntersectionGeometry->laneWidth) = refLaneWidth;
	// SpeedLimitList
	uint16_t refSpeedLimt = mapDataIn.mpApproaches[0].speed_limit;
	if (((pIntersectionGeometry->speedLimits = static_cast<SpeedLimitList_t *>(std::calloc(1, sizeof(SpeedLimitList_t)))) == NULL)
		|| ((pIntersectionGeometry->speedLimits->list.array = static_cast<RegulatorySpeedLimit_t **>(std::calloc(1, sizeof(RegulatorySpeedLimit_t *)))) == NULL)
		|| ((pIntersectionGeometry->speedLimits->list.array[0] = static_cast<RegulatorySpeedLimit_t *>(std::calloc(1, sizeof(RegulatorySpeedLimit_t)))) == NULL))
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".SpeedLimitList" << std::endl;
		return(false);
	}
	pIntersectionGeometry->speedLimits->list.size = 1;
	pIntersectionGeometry->speedLimits->list.count = 1;
	pIntersectionGeometry->speedLimits->list.array[0]->type  = SpeedLimitType_vehicleMaxSpeed;
	pIntersectionGeometry->speedLimits->list.array[0]->speed = refSpeedLimt;
	// LaneList
	std::string branch_level{".LaneList"};
	size_t num_lanes = 0;
	for (auto it = mapDataIn.mpApproaches.begin(); it != mapDataIn.mpApproaches.end(); ++it)
	{
		if (!it->mpLanes.empty())
			num_lanes += it->mpLanes.size();
	}
	if ((pIntersectionGeometry->laneSet.list.array = (GenericLane_t **)calloc(num_lanes, sizeof(GenericLane_t *))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << std::endl;
		return(false);
	}
	pIntersectionGeometry->laneSet.list.size = static_cast<int>(num_lanes);
	branch_level += ".GenericLane";
	int& laneListCnt = pIntersectionGeometry->laneSet.list.count;
	// loop through approaches / lanes
	bool has_error = false;  // for earlier return
	for (const auto& approachStruct : mapDataIn.mpApproaches)
	{
		for (const auto& laneStruct : approachStruct.mpLanes)
		{
			if ((pIntersectionGeometry->laneSet.list.array[laneListCnt] = (GenericLane_t *)calloc(1, sizeof(GenericLane_t))) == NULL)
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << std::endl;
				has_error = true;
				break;
			}
			GenericLane_t* pGenericLane = pIntersectionGeometry->laneSet.list.array[laneListCnt++];
			// GenericLane:
			// -- Required objects ------------------------------------ //
			//	LaneID
			//	LaneAttributes
			//	NodeListXY
			// -- OPTIONAL objects ------------ including/excluding  -- //
			//	DescriptiveName                     EXCL
			//	ApproachID (inbound/outbound)       INCL
			//	AllowedManeuvers                    INCL
			//	ConnectsToList                      INCL
			//	OverlayLaneList                     EXCL
			//	RegionalExtension                   EXCL
			// -------------------------------------------------------- //
			// LaneID
			pGenericLane->laneID = laneStruct.id;
			// LaneAttributes::LaneDirection - 2 bits BIT STRING
			std::bitset<2> direction_attributes;
			switch(approachStruct.type)
			{
			case MsgEnum::approachType::inbound:
				direction_attributes.set(0);
				break;
			case MsgEnum::approachType::outbound:
				direction_attributes.set(1);
				break;
			case MsgEnum::approachType::crosswalk:
				direction_attributes.set();
				break;
			}
			auto& laneDirection = pGenericLane->laneAttributes.directionalUse;
			if (!ul2bitString(&laneDirection.buf, laneDirection.size, laneDirection.bits_unused, 2, direction_attributes.to_ulong()))
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".LaneAttributes.LaneDirection" << std::endl;
				has_error = true;
				break;
			}
			// LaneAttributes::LaneSharing - 10 bits BIT STRING - 'not shared' and 'not overlapping'
			auto& laneSharing = pGenericLane->laneAttributes.sharedWith;
			if (!ul2bitString(&laneSharing.buf, laneSharing.size, laneSharing.bits_unused, 10, 0))
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".LaneAttributes.LaneSharing" << std::endl;
				has_error = true;
				break;
			}
			// LaneAttributes::LaneTypeAttributes
			auto& laneTypeAttributes = pGenericLane->laneAttributes.laneType;
			if (approachStruct.type == MsgEnum::approachType::crosswalk)
			{ // Crosswalk - 16 bits BIT STRING
				laneTypeAttributes.present = LaneTypeAttributes_PR_crosswalk;
				auto& crosswalk = laneTypeAttributes.choice.crosswalk;
				if (!ul2bitString(&crosswalk.buf, crosswalk.size, crosswalk.bits_unused, 16, laneStruct.attributes.to_ulong()))
				{
					std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".LaneAttributes.LaneTypeAttributes.Crosswalk" << std::endl;
					has_error = true;
					break;
				}
			}
			else
			{ // Vehicle - 8 bits BIT STRING
				laneTypeAttributes.present = LaneTypeAttributes_PR_vehicle;
				auto& vehicle = laneTypeAttributes.choice.vehicle;
				if (!ul2bitString(&vehicle.buf, vehicle.size, vehicle.bits_unused, 8, (laneStruct.attributes.to_ulong() & 0xFF)))
				{
					std::cerr << prog_name << "allocate " << allocate_level << branch_level << ".LaneAttributes.LaneTypeAttributes.Vehicle" << std::endl;
					has_error = true;
					break;
				}
			}
			// ApproachID
			switch(approachStruct.type)
			{
			case MsgEnum::approachType::outbound:
				if ((pGenericLane->egressApproach = (ApproachID_t *)calloc(1, sizeof(ApproachID_t))) == NULL)
					has_error = true;
				else
					*(pGenericLane->egressApproach) = approachStruct.id;
				break;
			case MsgEnum::approachType::inbound:
			case MsgEnum::approachType::crosswalk:
				if ((pGenericLane->ingressApproach = (ApproachID_t *)calloc(1, sizeof(ApproachID_t))) == NULL)
					has_error = true;
				else
					*(pGenericLane->ingressApproach) = approachStruct.id;
				break;
			}
			if (has_error)
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
				std::cerr << ".ApproachID" << std::endl;
				break;
			}
			// AllowedManeuvers - 12 bits BIT STRING
			if (approachStruct.type != MsgEnum::approachType::crosswalk)
			{
				if (((pGenericLane->maneuvers = (AllowedManeuvers_t *)calloc(1, sizeof(AllowedManeuvers_t))) == NULL)
					|| !(ul2bitString(&pGenericLane->maneuvers->buf, pGenericLane->maneuvers->size,
						pGenericLane->maneuvers->bits_unused, 12, (laneStruct.attributes.to_ulong() >> 8))))
				{
					std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".AllowedManeuvers" << std::endl;
					has_error = true;
					break;
				}
			}
			// ConnectsToList
			if (!laneStruct.mpConnectTo.empty())
			{
				if (((pGenericLane->connectsTo = (ConnectsToList_t *)calloc(1, sizeof(ConnectsToList_t))) == NULL)
					|| ((pGenericLane->connectsTo->list.array = (Connection_t **)calloc(laneStruct.mpConnectTo.size(), sizeof(Connection_t *))) == NULL))
				{
					std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".ConnectsToList" << std::endl;
					has_error = true;
					break;
				}
				pGenericLane->connectsTo->list.size = static_cast<int>(laneStruct.mpConnectTo.size());
				int& connListCnt = pGenericLane->connectsTo->list.count;
				for (const auto& connStruct : laneStruct.mpConnectTo)
				{
					if ((pGenericLane->connectsTo->list.array[connListCnt] = (Connection_t *)calloc(1, sizeof(Connection_t))) == NULL)
					{
						std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".ConnectsToList.Connection" << std::endl;
						has_error = true;
						break;
					}
					Connection_t* pConnection = pGenericLane->connectsTo->list.array[connListCnt++];
					// Connection:
					// -- Required objects ------------------------------------ //
					//	ConnectingLane
					// -- OPTIONAL objects ------------ including/excluding  -- //
					//	IntersectionReferenceID             INCL
					//	SignalGroupID                       INCL
					//	RestrictionClassID                  EXCL
					//	LaneConnectionID                    EXCL
					// -------------------------------------------------------- //
					auto& connLane = pConnection->connectingLane;
					// ConnectingLane
					// -- Required objects ------------------------------------ //
					//	LaneID
					// -- OPTIONAL objects ------------ including/excluding  -- //
					//	AllowedManeuvers                    INCL
					// -------------------------------------------------------- //
					connLane.lane = connStruct.laneId;
					if (connStruct.laneManeuver != MsgEnum::maneuverType::unavailable)
					{ // AllowedManeuvers - 12 bits BIT STRING
						std::bitset<12> connecting_maneuvers;
						switch(connStruct.laneManeuver)
						{
						case MsgEnum::maneuverType::uTurn:
							connecting_maneuvers.set(3);
							break;
						case MsgEnum::maneuverType::leftTurn:
							connecting_maneuvers.set(1);
							break;
						case MsgEnum::maneuverType::rightTurn:
							connecting_maneuvers.set(2);
							break;
						case MsgEnum::maneuverType::straightAhead:
						case MsgEnum::maneuverType::straight:
							connecting_maneuvers.set(0);
							break;
						default:
							break;
						}
						if (((connLane.maneuver = (AllowedManeuvers_t *)calloc(1, sizeof(AllowedManeuvers_t))) == NULL)
							|| !(ul2bitString(&connLane.maneuver->buf, connLane.maneuver->size,
								connLane.maneuver->bits_unused, 12, connecting_maneuvers.to_ulong())))
						{
							std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
							std::cerr << ".ConnectsToList.Connection.connectingLane.AllowedManeuvers" << std::endl;
							has_error = true;
							break;
						}
					}
					// IntersectionReferenceID
					if ((connStruct.intersectionId != mapDataIn.id) || (connStruct.regionalId != mapDataIn.regionalId))
					{
						if (((pConnection->remoteIntersection = (IntersectionReferenceID_t *)calloc(1, sizeof(IntersectionReferenceID_t))) == NULL)
							|| ((pConnection->remoteIntersection->region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t))) == NULL))
						{
							std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
							std::cerr << ".ConnectsToList.Connection.IntersectionReferenceID" << std::endl;
							has_error = true;
							break;
						}
						pConnection->remoteIntersection->id = connStruct.intersectionId;
						*(pConnection->remoteIntersection->region) = connStruct.regionalId;
					}
					// SignalGroupID
					if (laneStruct.controlPhase != 0)
					{
						if ((pConnection->signalGroup = (SignalGroupID_t *)calloc(1, sizeof(SignalGroupID_t))) == NULL)
						{
							std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
							std::cerr << ".ConnectsToList.Connection.signalGroup" << std::endl;
							has_error = true;
							break;
						}
						*(pConnection->signalGroup) = laneStruct.controlPhase;
					}
				} // end of loop through connectsTo
				if (has_error)
					break;
			}
			// NodeListXY
			pGenericLane->nodeList.present = NodeListXY_PR_nodes; // NodeSetXY
			auto& nodeSet = pGenericLane->nodeList.choice.nodes;
			if ((nodeSet.list.array = (NodeXY_t **)calloc(laneStruct.mpNodes.size(), sizeof(NodeXY_t *))) == NULL)
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".NodeListXY" << std::endl;
				has_error = true;
				break;
			}
			nodeSet.list.size = static_cast<int>(laneStruct.mpNodes.size());
			// NodeSetXY::NodeXY
			int& nodeListCnt = nodeSet.list.count;
			for (auto it = laneStruct.mpNodes.cbegin(); it != laneStruct.mpNodes.cend(); ++it)
			{
				if ((nodeSet.list.array[nodeListCnt] = (NodeXY_t *)calloc(1, sizeof(NodeXY_t))) == NULL)
				{
					std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".NodeListXY.NodeXY" << std::endl;
					has_error = true;
					break;
				}
				// NodeXY::NodeOffsetPointXY
				NodeXY_t* pNode = nodeSet.list.array[nodeListCnt++];
				if (it->useXY)
				{
					const auto& offset_x = it->offset_x;
					const auto& offset_y = it->offset_y;
					uint32_t offset_dist = static_cast<uint32_t>(std::sqrt(offset_x * offset_x + offset_y * offset_y));
					if (offset_dist <= 511)
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY1;
						pNode->delta.choice.node_XY1.x = offset_x;
						pNode->delta.choice.node_XY1.y = offset_y;
					}
					else if (offset_dist <= 1023)
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY2;
						pNode->delta.choice.node_XY2.x = offset_x;
						pNode->delta.choice.node_XY2.y = offset_y;
					}
					else if (offset_dist <= 2047)
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY3;
						pNode->delta.choice.node_XY3.x = offset_x;
						pNode->delta.choice.node_XY3.y = offset_y;
					}
					else if (offset_dist <= 4096)
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY4;
						pNode->delta.choice.node_XY4.x = offset_x;
						pNode->delta.choice.node_XY4.y = offset_y;
					}
					else if (offset_dist <= 8191)
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY5;
						pNode->delta.choice.node_XY5.x = offset_x;
						pNode->delta.choice.node_XY5.y = offset_y;
					}
					else
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_XY6;
						pNode->delta.choice.node_XY6.x = offset_x;
						pNode->delta.choice.node_XY6.y = offset_y;
					}
				}
				else
				{
					pNode->delta.present = NodeOffsetPointXY_PR_node_LatLon;
					pNode->delta.choice.node_LatLon.lat = it->latitude;
					pNode->delta.choice.node_LatLon.lon = it->longitude;
				}
				// NodeXY::NodeAttributeSetXY
				if (it == laneStruct.mpNodes.cbegin())
				{
					bool laneWidthAdjust  = (laneStruct.width != refLaneWidth);
					bool speedLimitAdjust = ((approachStruct.type != MsgEnum::approachType::crosswalk) && (approachStruct.speed_limit > 0)
						&& (approachStruct.speed_limit < MsgEnum::unknown_speed) && (approachStruct.speed_limit != refSpeedLimt));
					if (laneWidthAdjust || speedLimitAdjust)
					{
						if ((pNode->attributes = (NodeAttributeSetXY_t *)calloc(1, sizeof(NodeAttributeSetXY_t))) == NULL)
						{
							std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
							std::cerr << ".NodeListXY.NodeXY.NodeAttributeSetXY" << std::endl;
							has_error = true;
							break;
						}
						// lane width adjustment w.r.t. refLaneWidth
						if (laneWidthAdjust)
						{
							if ((pNode->attributes->dWidth = (Offset_B10_t *)calloc(1, sizeof(Offset_B10_t))) == NULL)
							{
								std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
								std::cerr << ".NodeListXY.NodeXY.NodeAttributeSetXY.dWidth" << std::endl;
								has_error = true;
								break;
							}
							*(pNode->attributes->dWidth) = laneStruct.width - refLaneWidth;
							refLaneWidth = laneStruct.width;
						}
						// speed limit
						if (speedLimitAdjust)
						{
							if (((pNode->attributes->data = (LaneDataAttributeList_t *)calloc(1, sizeof(LaneDataAttributeList_t))) == NULL)
								|| ((pNode->attributes->data->list.array = (LaneDataAttribute_t **)calloc(1, sizeof(LaneDataAttribute_t *))) == NULL)
								|| ((pNode->attributes->data->list.array[0] = (LaneDataAttribute_t *)calloc(1, sizeof(LaneDataAttribute_t))) == NULL))
							{
								std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
								std::cerr << ".NodeListXY.NodeXY.NodeAttributeSetXY.LaneDataAttribute" << std::endl;
								has_error = true;
								break;
							}
							pNode->attributes->data->list.size = 1;
							pNode->attributes->data->list.count = 1;
							LaneDataAttribute_t* pdata = pNode->attributes->data->list.array[0];
							pdata->present = LaneDataAttribute_PR_speedLimits;
							if (((pdata->choice.speedLimits.list.array = (RegulatorySpeedLimit_t **)calloc(1, sizeof(RegulatorySpeedLimit_t *))) == NULL)
								|| ((pdata->choice.speedLimits.list.array[0] = (RegulatorySpeedLimit_t *)calloc(1, sizeof(RegulatorySpeedLimit_t))) == NULL))
							{
								std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
								std::cerr << ".NodeListXY.NodeXY.NodeAttributeSetXY.LaneDataAttribute.RegulatorySpeedLimit" << std::endl;
								has_error = true;
								break;
							}
							pdata->choice.speedLimits.list.size = 1;
							pdata->choice.speedLimits.list.count = 1;
							pdata->choice.speedLimits.list.array[0]->type  = SpeedLimitType_vehicleMaxSpeed;
							pdata->choice.speedLimits.list.array[0]->speed = approachStruct.speed_limit;
							refSpeedLimt = approachStruct.speed_limit;
						}
					}
				}
			}
			if (has_error)
				break;
		}
		if (has_error)
			break;
	}
	return(!has_error);
};

/// fill MapData_t structure which specifies speed limit at the approach level
auto mapData2msgFrame_multi = [](const MapData_element_t& mapDataIn, MapData_t& mapData)->bool
{	// MapData:
	// -- Required objects ------------------------------------ //
	//	msgIssueRevision
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	MinuteOfTheYear                     EXCL
	//	LayerType                           INCL
	//	LayerID                             EXCL
	//	IntersectionGeometryList            INCL
	//	RoadSegmentList                     EXCL
	//	DataParameters                      EXCL
	//	RestrictionClassList                EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	std::string prog_name = "mapData2msgFrame_multi: ";
	std::string allocate_level{"MapData"};
	// msgIssueRevision
	mapData.msgIssueRevision	= mapDataIn.mapVersion;
	// LayerType
	if ((mapData.layerType = (LayerType_t *)calloc(1, sizeof(LayerType_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".LayerType" << std::endl;
		return(false);
	}
	*(mapData.layerType) = LayerType_intersectionData;
	// IntersectionGeometryList - one intersection per MapData, one IntersectionGeometry per distinct speed limit
	allocate_level += ".IntersectionGeometryList";
	if (((mapData.intersections = (IntersectionGeometryList_t *)calloc(1, sizeof(IntersectionGeometryList_t))) == NULL)
		|| ((mapData.intersections->list.array = (IntersectionGeometry_t **)calloc(mapDataIn.speeds.size(), sizeof(IntersectionGeometry_t *))) == NULL))
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << std::endl;
		return(false);
	}
	mapData.intersections->list.size = static_cast<int>(mapDataIn.speeds.size());
	// loop through distinct speed limits
	allocate_level += ".IntersectionGeometry";
	bool has_error = false;  // for earlier return
	int& geoListCnt = mapData.intersections->list.count;
	for (const auto& speed_limit : mapDataIn.speeds)
	{	// get indexes of approaches and number of lanes having the targeted speed limit
		std::vector<uint8_t> approachIndex;
		size_t num_lanes = 0;
		for (auto it = mapDataIn.mpApproaches.begin(); it != mapDataIn.mpApproaches.end(); ++it)
		{
			if ((it->speed_limit == speed_limit) && (!it->mpLanes.empty()))
			{
				approachIndex.push_back((uint8_t)(it - mapDataIn.mpApproaches.begin()));
				num_lanes += it->mpLanes.size();
			}
		}
		// get the reference lane width for this speed group
		uint16_t refLaneWidth = mapDataIn.mpApproaches[approachIndex[0]].mpLanes[0].width;
		// allocate IntersectionGeometry - one per speed group
		if ((mapData.intersections->list.array[geoListCnt] = (IntersectionGeometry_t *)calloc(1, sizeof(IntersectionGeometry_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << std::endl;
			has_error = true;
			break;
		}
		IntersectionGeometry_t* pIntersectionGeometry = mapData.intersections->list.array[geoListCnt];
		// IntersectionGeometry:
		// -- Required objects ------------------------------------ //
		//	IntersectionReferenceID
		//	MsgCount
		//	Position3D
		//	LaneList
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	DescriptiveName                     EXCL
		//	LaneWidth                           INCL
		//	SpeedLimitList                      INCL
		//	PreemptPriorityList                 EXCL
		//	RegionalExtension                   EXCL
		// -------------------------------------------------------- //
		// IntersectionReferenceID:
		// -- Required objects ------------------------------------ //
		//	IntersectionID
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	RoadRegulatorID                     INCL
		// -------------------------------------------------------- //
		pIntersectionGeometry->id.id = mapDataIn.id;
		if ((pIntersectionGeometry->id.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".id.region" << std::endl;
			has_error = true;
			break;
		}
		*(pIntersectionGeometry->id.region) = mapDataIn.regionalId;
		// MsgCount
		pIntersectionGeometry->revision = geoListCnt++;
		// Position3D
		pIntersectionGeometry->refPoint.lat = mapDataIn.geoRef.latitude;
		pIntersectionGeometry->refPoint.Long = mapDataIn.geoRef.longitude;
		if (mapDataIn.attributes.test(0))
		{ // include elevation data
			if ((pIntersectionGeometry->refPoint.elevation = (DSRC_Elevation_t *)calloc(1, sizeof(DSRC_Elevation_t))) == NULL)
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << ".Position3D.Elevation" << std::endl;
				has_error = true;
				break;
			}
			*(pIntersectionGeometry->refPoint.elevation) = mapDataIn.geoRef.elevation;
		}
		// LaneWidth
		if ((pIntersectionGeometry->laneWidth = (LaneWidth_t *)calloc(1, sizeof(LaneWidth_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".LaneWidth" << std::endl;
			has_error = true;
			break;
		}
		*(pIntersectionGeometry->laneWidth) = refLaneWidth;
		// SpeedLimitList
		if ((speed_limit > 0) && (speed_limit < MsgEnum::unknown_speed))
		{ // 0 = crosswalk, MsgEnum::unknown_speed = speed limit not available on vehicular lanes
			if (((pIntersectionGeometry->speedLimits = (SpeedLimitList_t *)calloc(1, sizeof(SpeedLimitList_t))) == NULL)
				|| ((pIntersectionGeometry->speedLimits->list.array = (RegulatorySpeedLimit_t **)calloc(1, sizeof(RegulatorySpeedLimit_t *))) == NULL)
				|| ((pIntersectionGeometry->speedLimits->list.array[0] = (RegulatorySpeedLimit_t *)calloc(1, sizeof(RegulatorySpeedLimit_t))) == NULL))
			{
				std::cerr << prog_name << "failed allocate " << allocate_level;
				std::cerr << ".SpeedLimitList.RegulatorySpeedLimit" << std::endl;
				has_error = true;
				break;
			}
			pIntersectionGeometry->speedLimits->list.size = 1;
			pIntersectionGeometry->speedLimits->list.count = 1;
			pIntersectionGeometry->speedLimits->list.array[0]->type  = SpeedLimitType_vehicleMaxSpeed;
			pIntersectionGeometry->speedLimits->list.array[0]->speed = speed_limit;
		}
		// LaneList
		std::string branch_level{".LaneList"};
		if ((pIntersectionGeometry->laneSet.list.array = (GenericLane_t **)calloc(num_lanes, sizeof(GenericLane_t *))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << std::endl;
			has_error = true;
			break;
		}
		pIntersectionGeometry->laneSet.list.size = static_cast<int>(num_lanes);
		branch_level += ".GenericLane";
		int& laneListCnt = pIntersectionGeometry->laneSet.list.count;
		// loop through approaches / lanes
		for (const auto& i_approach : approachIndex)
		{
			const auto& approachStruct = mapDataIn.mpApproaches[i_approach];
			for (const auto& laneStruct : approachStruct.mpLanes)
			{
				if ((pIntersectionGeometry->laneSet.list.array[laneListCnt] = (GenericLane_t *)calloc(1, sizeof(GenericLane_t))) == NULL)
				{
					std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << std::endl;
					has_error = true;
					break;
				}
				GenericLane_t* pGenericLane = pIntersectionGeometry->laneSet.list.array[laneListCnt++];
				// GenericLane:
				// -- Required objects ------------------------------------ //
				//	LaneID
				//	LaneAttributes
				//	NodeListXY
				// -- OPTIONAL objects ------------ including/excluding  -- //
				//	DescriptiveName                     EXCL
				//	ApproachID (inbound/outbound)       INCL
				//	AllowedManeuvers                    INCL
				//	ConnectsToList                      INCL
				//	OverlayLaneList                     EXCL
				//	RegionalExtension                   EXCL
				// -------------------------------------------------------- //
				// LaneID
				pGenericLane->laneID = laneStruct.id;
				// LaneAttributes::LaneDirection - 2 bits BIT STRING
				std::bitset<2> direction_attributes;
				switch(approachStruct.type)
				{
				case MsgEnum::approachType::inbound:
					direction_attributes.set(0);
					break;
				case MsgEnum::approachType::outbound:
					direction_attributes.set(1);
					break;
				case MsgEnum::approachType::crosswalk:
					direction_attributes.set();
					break;
				}
				auto& laneDirection = pGenericLane->laneAttributes.directionalUse;
				if (!ul2bitString(&laneDirection.buf, laneDirection.size, laneDirection.bits_unused, 2, direction_attributes.to_ulong()))
				{
					std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".LaneAttributes.LaneDirection" << std::endl;
					has_error = true;
					break;
				}
				// LaneAttributes::LaneSharing - 10 bits BIT STRING - 'not shared' and 'not overlapping'
				auto& laneSharing = pGenericLane->laneAttributes.sharedWith;
				if (!ul2bitString(&laneSharing.buf, laneSharing.size, laneSharing.bits_unused, 10, 0))
				{
					std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
					std::cerr << ".LaneAttributes.LaneSharing" << std::endl;
					has_error = true;
					break;
				}
				// LaneAttributes::LaneTypeAttributes
				auto& laneTypeAttributes = pGenericLane->laneAttributes.laneType;
				if (approachStruct.type == MsgEnum::approachType::crosswalk)
				{ // Crosswalk - 16 bits BIT STRING
					laneTypeAttributes.present = LaneTypeAttributes_PR_crosswalk;
					auto& crosswalk = laneTypeAttributes.choice.crosswalk;
					if (!ul2bitString(&crosswalk.buf, crosswalk.size, crosswalk.bits_unused, 16, laneStruct.attributes.to_ulong()))
					{
						std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
						std::cerr << ".LaneAttributes.LaneTypeAttributes.Crosswalk" << std::endl;
						has_error = true;
						break;
					}
				}
				else
				{ // Vehicle - 8 bits BIT STRING
					laneTypeAttributes.present = LaneTypeAttributes_PR_vehicle;
					auto& vehicle = laneTypeAttributes.choice.vehicle;
					if (!ul2bitString(&vehicle.buf, vehicle.size, vehicle.bits_unused, 8, (laneStruct.attributes.to_ulong() & 0xFF)))
					{
						std::cerr << prog_name << "allocate " << allocate_level << branch_level;
						std::cerr << ".LaneAttributes.LaneTypeAttributes.Vehicle" << std::endl;
						has_error = true;
						break;
					}
				}
				// ApproachID
				switch(approachStruct.type)
				{
				case MsgEnum::approachType::outbound:
					if ((pGenericLane->egressApproach = (ApproachID_t *)calloc(1, sizeof(ApproachID_t))) == NULL)
						has_error = true;
					else
						*(pGenericLane->egressApproach) = approachStruct.id;
					break;
				case MsgEnum::approachType::inbound:
				case MsgEnum::approachType::crosswalk:
					if ((pGenericLane->ingressApproach = (ApproachID_t *)calloc(1, sizeof(ApproachID_t))) == NULL)
						has_error = true;
					else
						*(pGenericLane->ingressApproach) = approachStruct.id;
					break;
				}
				if (has_error)
				{
					std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".ApproachID" << std::endl;
					break;
				}
				// AllowedManeuvers - 12 bits BIT STRING
				if (approachStruct.type != MsgEnum::approachType::crosswalk)
				{
					if (((pGenericLane->maneuvers = (AllowedManeuvers_t *)calloc(1, sizeof(AllowedManeuvers_t))) == NULL)
						|| !(ul2bitString(&pGenericLane->maneuvers->buf, pGenericLane->maneuvers->size,
							pGenericLane->maneuvers->bits_unused, 12, (laneStruct.attributes.to_ulong() >> 8))))
					{
						std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".AllowedManeuvers" << std::endl;
						has_error = true;
						break;
					}
				}
				// ConnectsToList
				if (!laneStruct.mpConnectTo.empty())
				{
					if (((pGenericLane->connectsTo = (ConnectsToList_t *)calloc(1, sizeof(ConnectsToList_t))) == NULL)
						|| ((pGenericLane->connectsTo->list.array = (Connection_t **)calloc(laneStruct.mpConnectTo.size(), sizeof(Connection_t *))) == NULL))
					{
						std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".ConnectsToList" << std::endl;
						has_error = true;
						break;
					}
					pGenericLane->connectsTo->list.size = static_cast<int>(laneStruct.mpConnectTo.size());
					int& connListCnt = pGenericLane->connectsTo->list.count;
					for (const auto& connStruct : laneStruct.mpConnectTo)
					{
						if ((pGenericLane->connectsTo->list.array[connListCnt] = (Connection_t *)calloc(1, sizeof(Connection_t))) == NULL)
						{
							std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".ConnectsToList.Connection" << std::endl;
							has_error = true;
							break;
						}
						Connection_t* pConnection = pGenericLane->connectsTo->list.array[connListCnt++];
						// Connection:
						// -- Required objects ------------------------------------ //
						//	ConnectingLane
						// -- OPTIONAL objects ------------ including/excluding  -- //
						//	IntersectionReferenceID             INCL
						//	SignalGroupID                       INCL
						//	RestrictionClassID                  EXCL
						//	LaneConnectionID                    EXCL
						// -------------------------------------------------------- //

						auto& connLane = pConnection->connectingLane;
						// ConnectingLane
						// -- Required objects ------------------------------------ //
						//	LaneID
						// -- OPTIONAL objects ------------ including/excluding  -- //
						//	AllowedManeuvers                    INCL
						// -------------------------------------------------------- //
						connLane.lane = connStruct.laneId;
						if (connStruct.laneManeuver != MsgEnum::maneuverType::unavailable)
						{ // AllowedManeuvers - 12 bits BIT STRING
							std::bitset<12> connecting_maneuvers;
							switch(connStruct.laneManeuver)
							{
							case MsgEnum::maneuverType::uTurn:
								connecting_maneuvers.set(3);
								break;
							case MsgEnum::maneuverType::leftTurn:
								connecting_maneuvers.set(1);
								break;
							case MsgEnum::maneuverType::rightTurn:
								connecting_maneuvers.set(2);
								break;
							case MsgEnum::maneuverType::straightAhead:
							case MsgEnum::maneuverType::straight:
								connecting_maneuvers.set(0);
								break;
							default:
								break;
							}
							if (((connLane.maneuver = (AllowedManeuvers_t *)calloc(1, sizeof(AllowedManeuvers_t))) == NULL)
								|| !(ul2bitString(&connLane.maneuver->buf, connLane.maneuver->size,
									connLane.maneuver->bits_unused, 12, connecting_maneuvers.to_ulong())))
							{
								std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
								std::cerr << ".ConnectsToList.Connection.connectingLane.AllowedManeuvers" << std::endl;
								has_error = true;
								break;
							}
						}
						// IntersectionReferenceID
						if ((connStruct.intersectionId != mapDataIn.id) || (connStruct.regionalId != mapDataIn.regionalId))
						{
							if (((pConnection->remoteIntersection = (IntersectionReferenceID_t *)calloc(1, sizeof(IntersectionReferenceID_t))) == NULL)
								|| ((pConnection->remoteIntersection->region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t))) == NULL))
							{
								std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
								std::cerr << ".ConnectsToList.Connection.IntersectionReferenceID" << std::endl;
								has_error = true;
								break;
							}
							pConnection->remoteIntersection->id = connStruct.intersectionId;
							*(pConnection->remoteIntersection->region) = connStruct.regionalId;
						}
						// SignalGroupID
						if (laneStruct.controlPhase != 0)
						{
							if ((pConnection->signalGroup = (SignalGroupID_t *)calloc(1, sizeof(SignalGroupID_t))) == NULL)
							{
								std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
								std::cerr << ".ConnectsToList.Connection.signalGroup" << std::endl;
								has_error = true;
								break;
							}
							*(pConnection->signalGroup) = laneStruct.controlPhase;
						}
					} // end of loop through connectsTo
					if (has_error)
						break;
				}
				// NodeListXY
				pGenericLane->nodeList.present = NodeListXY_PR_nodes; // NodeSetXY
				auto& nodeSet = pGenericLane->nodeList.choice.nodes;
				if ((nodeSet.list.array = (NodeXY_t **)calloc(laneStruct.mpNodes.size(), sizeof(NodeXY_t *))) == NULL)
				{
					std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".NodeListXY" << std::endl;
					has_error = true;
					break;
				}
				nodeSet.list.size = static_cast<int>(laneStruct.mpNodes.size());
				// NodeSetXY::NodeXY
				int& nodeListCnt = nodeSet.list.count;
				for (size_t i_node = 0; i_node < laneStruct.mpNodes.size(); i_node++)
				{
					const auto& nodeStruct = laneStruct.mpNodes[i_node];
					if ((nodeSet.list.array[nodeListCnt] = (NodeXY_t *)calloc(1, sizeof(NodeXY_t))) == NULL)
					{
						std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".NodeListXY.NodeXY" << std::endl;
						has_error = true;
						break;
					}
					// NodeXY::NodeOffsetPointXY
					NodeXY_t* pNode = nodeSet.list.array[nodeListCnt++];
					if (nodeStruct.useXY)
					{
						const auto& offset_x = nodeStruct.offset_x;
						const auto& offset_y = nodeStruct.offset_y;
						uint32_t offset_dist = static_cast<uint32_t>(std::sqrt(offset_x * offset_x + offset_y * offset_y));
						if (offset_dist <= 511)
						{
							pNode->delta.present = NodeOffsetPointXY_PR_node_XY1;
							pNode->delta.choice.node_XY1.x = offset_x;
							pNode->delta.choice.node_XY1.y = offset_y;
						}
						else if (offset_dist <= 1023)
						{
							pNode->delta.present = NodeOffsetPointXY_PR_node_XY2;
							pNode->delta.choice.node_XY2.x = offset_x;
							pNode->delta.choice.node_XY2.y = offset_y;
						}
						else if (offset_dist <= 2047)
						{
							pNode->delta.present = NodeOffsetPointXY_PR_node_XY3;
							pNode->delta.choice.node_XY3.x = offset_x;
							pNode->delta.choice.node_XY3.y = offset_y;
						}
						else if (offset_dist <= 4096)
						{
							pNode->delta.present = NodeOffsetPointXY_PR_node_XY4;
							pNode->delta.choice.node_XY4.x = offset_x;
							pNode->delta.choice.node_XY4.y = offset_y;
						}
						else if (offset_dist <= 8191)
						{
							pNode->delta.present = NodeOffsetPointXY_PR_node_XY5;
							pNode->delta.choice.node_XY5.x = offset_x;
							pNode->delta.choice.node_XY5.y = offset_y;
						}
						else
						{
							pNode->delta.present = NodeOffsetPointXY_PR_node_XY6;
							pNode->delta.choice.node_XY6.x = offset_x;
							pNode->delta.choice.node_XY6.y = offset_y;
						}
					}
					else
					{
						pNode->delta.present = NodeOffsetPointXY_PR_node_LatLon;
						pNode->delta.choice.node_LatLon.lat = nodeStruct.latitude;
						pNode->delta.choice.node_LatLon.lon = nodeStruct.longitude;
					}
					// NodeXY::NodeAttributeSetXY - lane width adjustment w.r.t. refLaneWidth
					if ((laneStruct.width != refLaneWidth) && (i_node == 0))
					{
						if (((pNode->attributes = (NodeAttributeSetXY_t *)calloc(1, sizeof(NodeAttributeSetXY_t))) == NULL)
							|| ((pNode->attributes->dWidth = (Offset_B10_t *)calloc(1, sizeof(Offset_B10_t))) == NULL))
						{
							std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
							std::cerr << ".NodeListXY.NodeXY.NodeAttributeSetXY" << std::endl;
							has_error = true;
							break;
						}
						*(pNode->attributes->dWidth) = laneStruct.width - refLaneWidth;
					}
				}
				if (has_error)
					break;
			}
			if (has_error)
				break;
		}
		if (has_error)
			break;
	}
	return(!has_error);
};

/// fill SPAT_t structure
auto spat2msgFrame = [](const SPAT_element_t& spatIn, SPAT_t& spat)->bool
{ // get array of signalGroupID for permitted vehicular and pedestrian phases
	std::vector<int> signalGroupArray;
	for (uint8_t i = 0; i < 8; i++)
	{ // vehicular phases
		if (spatIn.permittedPhases.test(i))
			signalGroupArray.push_back(i);
	}
	for (uint8_t i = 0; i < 8; i++)
	{ // pedestrian phases
		if (spatIn.permittedPedPhases.test(i))
			signalGroupArray.push_back(i + 8);
	}
	if (signalGroupArray.empty())
		return(false);   // nothing to encode
	// SPAT:
	// -- Required objects ------------------------------------ //
	//	IntersectionStateList
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	MinuteOfTheYear                     EXCL
	//	DescriptiveName                     EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	// IntersectionStateList - one intersection per SPAT
	std::string prog_name = "spat2msgFrame: ";
	std::string allocate_level{"SPAT.IntersectionStateList.IntersectionState"};
	if (((spat.intersections.list.array = (IntersectionState_t **)calloc(1, sizeof(IntersectionState_t *))) == NULL)
		|| ((spat.intersections.list.array[0] = (IntersectionState_t *)calloc(1, sizeof(IntersectionState_t))) == NULL))
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << std::endl;
		return(false);
	}
	spat.intersections.list.size  = 1;
	spat.intersections.list.count = 1;
	IntersectionState_t* pIntsectionState = spat.intersections.list.array[0];
	// IntersectionStateList::IntersectionState
	// -- Required objects ------------------------------------ //
	//	IntersectionReferenceID
	//	MsgCount
	//	IntersectionStatusObject
	//	MovementList
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	DescriptiveName                     EXCL
	//	MinuteOfTheYear                     OPTIONAL
	//	DSecond                             OPTIONAL
	//	EnabledLaneList                     EXCL
	//	ManeuverAssistList                  EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	// IntersectionReferenceID:
	// -- Required objects ------------------------------------ //
	//	IntersectionID
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	RoadRegulatorID                     INCL
	// -------------------------------------------------------- //
	pIntsectionState->id.id = spatIn.id;
	if ((pIntsectionState->id.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".id.region" << std::endl;
		return(false);
	}
	*(pIntsectionState->id.region) = spatIn.regionalId;
	// MsgCount
	pIntsectionState->revision = spatIn.msgCnt;
	// IntersectionStatusObject - 16 bits BIT STRING
	if (!ul2bitString(&pIntsectionState->status.buf, pIntsectionState->status.size,
		pIntsectionState->status.bits_unused, 16, spatIn.status.to_ulong()))
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".IntersectionStatusObject" << std::endl;
		return(false);
	}
	// TimeStamp
	if (spatIn.timeStampMinute < MsgEnum::invalid_timeStampMinute)
	{
		if ((pIntsectionState->moy = (MinuteOfTheYear_t *)calloc(1, sizeof(MinuteOfTheYear_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".MinuteOfTheYear" << std::endl;
			return(false);
		}
		*(pIntsectionState->moy) = spatIn.timeStampMinute;
	}
	if (spatIn.timeStampSec < 0xFFFF)
	{
		if ((pIntsectionState->timeStamp = (DSecond_t *)calloc(1, sizeof(DSecond_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".timeStamp" << std::endl;
			return(false);
		}
		*(pIntsectionState->timeStamp) = spatIn.timeStampSec;
	}
	// MovementList
	allocate_level += ".MovementList"; // one MovementState per vehicular/pedestrian signal group
	if ((pIntsectionState->states.list.array =
		(MovementState_t **)calloc(signalGroupArray.size(), sizeof(MovementState_t *))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << std::endl;
		return(false);
	}
	pIntsectionState->states.list.size  = static_cast<int>(signalGroupArray.size());
	// loop through permitted movements
	allocate_level += ".MovementState";
	bool has_error = false;  // for earlier return
	int& stateListCnt = pIntsectionState->states.list.count;
	for (const auto& signal_group : signalGroupArray)
	{
		const PhaseState_element_t& phaseState = (signal_group < 8) ?
			spatIn.phaseState[signal_group] : spatIn.pedPhaseState[signal_group - 8];
		// allocate MovementState object
		if ((pIntsectionState->states.list.array[stateListCnt] = (MovementState_t *)calloc(1, sizeof(MovementState_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << std::endl;
			has_error = true;
			break;
		}
		MovementState_t* pMovementState = pIntsectionState->states.list.array[stateListCnt++];
		// MovementState
		// -- Required objects ------------------------------------ //
		//	SignalGroupID
		//	MovementEventList
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	DescriptiveName                     EXCL
		//	ManeuverAssistList                  EXCL
		//	RegionalExtension                   EXCL
		// -------------------------------------------------------- //
		// SignalGroupID
		pMovementState->signalGroup = signal_group + 1;
		// MovementEventList - one MovementEvent per movement
		std::string branch_level(".MovementEventList.MovementEvent");
		if (((pMovementState->state_time_speed.list.array = (MovementEvent_t **)calloc(1, sizeof(MovementEvent_t *))) == NULL)
			|| ((pMovementState->state_time_speed.list.array[0] = (MovementEvent_t *)calloc(1, sizeof(MovementEvent_t))) == NULL))
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << std::endl;
			has_error = true;
			break;
		}
		pMovementState->state_time_speed.list.size  = 1;
		pMovementState->state_time_speed.list.count = 1;
		MovementEvent_t* pMovementEvent = pMovementState->state_time_speed.list.array[0];
		// MovementEvent
		// -- Required objects ------------------------------------ //
		//	MovementPhaseState
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	TimeChangeDetails                   OPTIONAL
		//	AdvisorySpeedList                   EXCL
		//	RegionalExtension                   EXCL
		// -------------------------------------------------------- //
		// MovementPhaseState
		pMovementEvent->eventState = static_cast<MovementPhaseState_t>(phaseState.currState);
		branch_level += ".TimeChangeDetails";
		// TimeChangeDetails
		// -- Required objects ------------------------------------ //
		// 	minEndTime
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	startTime                           OPTIONAL
		//	maxEndTime                          OPTIONAL
		//	likelyTime                          EXCL
		//	confidence (on likelyTime)          EXCL
		//	nextTime                            EXCL
		// -------------------------------------------------------- //
		if (phaseState.minEndTime < MsgEnum::unknown_timeDetail)
		{
			if ((pMovementEvent->timing = (TimeChangeDetails *)calloc(1, sizeof(TimeChangeDetails))) == NULL)
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << std::endl;
				has_error = true;
				break;
			}
			// minEndTime
			pMovementEvent->timing->minEndTime = phaseState.minEndTime;
			// startTime
			if (phaseState.startTime < MsgEnum::unknown_timeDetail)
			{
				if ((pMovementEvent->timing->startTime = (DSRC_TimeMark_t *)calloc(1, sizeof(DSRC_TimeMark_t))) == NULL)
				{
					std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".startTime" << std::endl;
					has_error = true;
					break;
				}
				*(pMovementEvent->timing->startTime) = phaseState.startTime;
			}
			// maxEndTime
			if (phaseState.maxEndTime < MsgEnum::unknown_timeDetail)
			{
				if ((pMovementEvent->timing->maxEndTime = (DSRC_TimeMark_t *)calloc(1, sizeof(DSRC_TimeMark_t))) == NULL)
				{
					std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".maxEndTime" << std::endl;
					has_error = true;
					break;
				}
				*(pMovementEvent->timing->maxEndTime) = phaseState.maxEndTime;
			}
		}
	}
	return(!has_error);
};

/// fill BasicSafetyMessage_t structure
auto bsm2msgFrame = [](const BSM_element_t& bsmIn, BasicSafetyMessage_t& bsm)->bool
{	// BSM:
	// -- Required objects ------------------------------------ //
	//	BSMcoreData
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	partII                              EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	std::string prog_name{"bsm2msgFrame: "};
	std::string allocate_level{"BSM.BSMcoreData"};
	auto& coreData = bsm.coreData;
	coreData.msgCnt = bsmIn.msgCnt;
	// TemporaryID
	if (!id2temporaryId(&coreData.id.buf, coreData.id.size, bsmIn.id))
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".TemporaryID" << std::endl;
		return(false);
	}
	coreData.secMark = bsmIn.timeStampSec;
	coreData.lat = bsmIn.latitude;
	coreData.Long = bsmIn.longitude;
	coreData.elev = bsmIn.elevation;
	coreData.accuracy.semiMajor = bsmIn.semiMajor;
	coreData.accuracy.semiMinor = bsmIn.semiMinor;
	coreData.accuracy.orientation = bsmIn.orientation;
	coreData.transmission = static_cast<TransmissionState_t>(bsmIn.transState);
	coreData.speed = bsmIn.speed;
	coreData.heading = bsmIn.heading;
	coreData.angle = bsmIn.steeringAngle;
	coreData.accelSet.Long = bsmIn.accelLon;
	coreData.accelSet.lat = bsmIn.accelLat;
	coreData.accelSet.vert = bsmIn.accelVert;
	coreData.accelSet.yaw = bsmIn.yawRate;
	coreData.size.width = bsmIn.vehWidth;
	coreData.size.length = bsmIn.vehLen;
	allocate_level += ".BrakeSystemStatus";
	auto& brakeSystemStatus = coreData.brakes;
	auto& wheelBrakes = brakeSystemStatus.wheelBrakes;  // SIZE (5) BIT STRING
	if (!ul2bitString(&wheelBrakes.buf, wheelBrakes.size, wheelBrakes.bits_unused, 5, bsmIn.brakeAppliedStatus.to_ulong()))
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".BrakeAppliedStatus" << std::endl;
		return(false);
	}
	brakeSystemStatus.traction = static_cast<TractionControlStatus_t>(bsmIn.tractionControlStatus);
	brakeSystemStatus.abs = static_cast<AntiLockBrakeStatus_t>(bsmIn.absStatus);
	brakeSystemStatus.scs = static_cast<StabilityControlStatus_t>(bsmIn.stabilityControlStatus);
	brakeSystemStatus.brakeBoost = static_cast<BrakeBoostApplied_t>(bsmIn.brakeBoostApplied);
	brakeSystemStatus.auxBrakes = static_cast<AuxiliaryBrakeStatus_t>(bsmIn.auxiliaryBrakeStatus);
	return(true);
};

/// fill RTCMcorrections_t structure
auto rtcm2msgFrame = [](const RTCM_element_t& rtcmIn, RTCMcorrections_t& rtcm)->bool
{
	if (rtcmIn.payload.empty())
		return(false);  // nothing to encode
	// RTCMcorrections:
	// -- Required objects ------------------------------------ //
	//	MsgCount
	//	RTCM-Revision
	//	RTCMmessageList
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	MinuteOfTheYear                     OPTIONAL
	//	FullPositionVector                  EXCL
	//	RTCMheader                          EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	std::string prog_name{"rtcm2msgFrame: "};
	std::string allocate_level{"RTCM"};
	rtcm.msgCnt = rtcmIn.msgCnt;
	rtcm.rev = rtcmIn.rev;
	// MinuteOfTheYear
	if (rtcmIn.timeStampMinute < MsgEnum::invalid_timeStampMinute)
	{
		if ((rtcm.timeStamp = (MinuteOfTheYear_t *)calloc(1, sizeof(MinuteOfTheYear_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".MinuteOfTheYear" << std::endl;
			return(false);
		}
		*(rtcm.timeStamp) = rtcmIn.timeStampMinute;
	}
	// RTCMmessageList - one payload per RTCM
	allocate_level += ".RTCMmessageList.RTCMmessage";
	if (((rtcm.msgs.list.array = (RTCMmessage_t **)calloc(1, sizeof(RTCMmessage_t *))) == NULL)
		|| ((rtcm.msgs.list.array[0] = (RTCMmessage_t *)calloc(1, sizeof(RTCMmessage_t))) == NULL))
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << std::endl;
		return(false);
	}
	rtcm.msgs.list.size  = 1;
	rtcm.msgs.list.count = 1;
	RTCMmessage_t* pRTCMmessage = rtcm.msgs.list.array[0];
	if (OCTET_STRING_fromBuf(pRTCMmessage, reinterpret_cast<const char*>(&rtcmIn.payload[0]), (int)rtcmIn.payload.size()) < 0)
	{
		std::cerr << prog_name << "failed OCTET_STRING_fromBuf " << std::endl;
		return(false);
	}
	return(true);
};

/// fill SignalRequestMessage_t structure
auto srm2msgFrame = [](const SRM_element_t& srmIn, SignalRequestMessage_t& srm)->bool
{
	std::string prog_name{"srm2msgFrame: "};
	if ((srmIn.inApprochId == 0) && (srmIn.inLaneId == 0))
	{
		std::cerr << prog_name << "either entry approachID or laneID shall be specified" << std::endl;
		return(false);
	}
	// SRM:
	// -- Required objects ------------------------------------ //
	//	DSecond
	//	RequestorDescription
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	MinuteOfTheYear                     OPTIONAL
	//	MsgCount                            OPTIONAL
	//	SignalRequestList                   INCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	std::string allocate_level{"SRM"};
	// DSecond
	srm.second = srmIn.timeStampSec;
	// MinuteOfTheYear
	if (srmIn.timeStampMinute < MsgEnum::invalid_timeStampMinute)
	{
		if ((srm.timeStamp = (MinuteOfTheYear_t *)calloc(1, sizeof(MinuteOfTheYear_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".MinuteOfTheYear" << std::endl;
			return(false);
		}
		*(srm.timeStamp) = srmIn.timeStampMinute;
	}
	// MsgCount
	if (srmIn.msgCnt < 0xFF)
	{
		if ((srm.sequenceNumber = (DSRC_MsgCount_t *)calloc(1, sizeof(DSRC_MsgCount_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".MsgCount" << std::endl;
			return(false);
		}
		*(srm.sequenceNumber) = srmIn.msgCnt;
	}
	// SignalRequestList - request for one intersection
	std::string branch_level(".SignalRequestList.SignalRequestPackage");
	if (((srm.requests = (SignalRequestList_t *)calloc(1, sizeof(SignalRequestList_t))) == NULL)
		|| ((srm.requests->list.array = (SignalRequestPackage_t **)calloc(1, sizeof(SignalRequestPackage_t *))) == NULL)
		|| ((srm.requests->list.array[0] = (SignalRequestPackage_t *)calloc(1, sizeof(SignalRequestPackage_t))) == NULL))
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << std::endl;
		return(false);
	}
	srm.requests->list.size = 1;
	srm.requests->list.count = 1;
	SignalRequestPackage_t* pSignalRequestPackage = srm.requests->list.array[0];
	// SignalRequestPackage
	// -- Required objects ------------------------------------ //
	//	SignalRequest
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	minute (ETA.minutes)                OPTIONAL
	//	second (ETA.milliseconds)           OPTIONAL
	//	duration                            OPTIONAL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	// ETA
	if (srmIn.ETAminute < MsgEnum::invalid_timeStampMinute)
	{
		if ((pSignalRequestPackage->minute = (MinuteOfTheYear_t *)calloc(1, sizeof(MinuteOfTheYear_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".ETAminute" << std::endl;
			return(false);
		}
		*(pSignalRequestPackage->minute) = srmIn.ETAminute;
	}
	if (srmIn.ETAsec < 0xFFFF)
	{
		if ((pSignalRequestPackage->second = (DSecond_t *)calloc(1, sizeof(DSecond_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".ETAsec" << std::endl;
			return(false);
		}
		*(pSignalRequestPackage->second) = srmIn.ETAsec;
	}
	// duration
	if (srmIn.duration < 0xFFFF)
	{
		if ((pSignalRequestPackage->duration = (DSecond_t *)calloc(1, sizeof(DSecond_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << branch_level << ".duration" << std::endl;
			return(false);
		}
		*(pSignalRequestPackage->duration) = srmIn.duration;
	}
	auto& signalRequest = pSignalRequestPackage->request;
	// SignalRequest
	// -- Required objects ------------------------------------ //
	//	IntersectionReferenceID
	//	RequestID
	//	PriorityRequestType
	//	inBoundLane
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	outBoundLane                        OPTIONAL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	// IntersectionReferenceID:
	// -- Required objects ------------------------------------ //
	//	IntersectionID
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	RoadRegulatorID                     INCL
	// -------------------------------------------------------- //
	signalRequest.id.id = srmIn.intId;
	if ((signalRequest.id.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
		std::cerr << ".SignalRequest.SignalRequest.id.region" << std::endl;
		return(false);
	}
	*(signalRequest.id.region) = srmIn.regionalId;
	// RequestID
	signalRequest.requestID = srmIn.reqId;
	// PriorityRequestType
	signalRequest.requestType = static_cast<PriorityRequestType_t>(srmIn.reqType);
	// inBoundLane
	if (srmIn.inLaneId == 0)
	{
		signalRequest.inBoundLane.present = IntersectionAccessPoint_PR_approach;
		signalRequest.inBoundLane.choice.approach = srmIn.inApprochId;
	}
	else
	{
		signalRequest.inBoundLane.present = IntersectionAccessPoint_PR_lane;
		signalRequest.inBoundLane.choice.lane = srmIn.inLaneId;
	}
	// outBoundLane
	if (!((srmIn.outApproachId == 0) && (srmIn.outLaneId == 0)))
	{
		if ((signalRequest.outBoundLane = (IntersectionAccessPoint_t *)calloc(1, sizeof(IntersectionAccessPoint_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << branch_level;
			std::cerr << ".SignalRequest.outBoundLane" << std::endl;
			return(false);
		}
		if (srmIn.outLaneId == 0)
		{
			signalRequest.outBoundLane->present = IntersectionAccessPoint_PR_approach;
			signalRequest.outBoundLane->choice.approach = srmIn.outApproachId;
		}
		else
		{
			signalRequest.outBoundLane->present = IntersectionAccessPoint_PR_lane;
			signalRequest.outBoundLane->choice.lane = srmIn.outLaneId;
		}
	}
	// RequestorDescription
	allocate_level += ".RequestorDescription";
	auto& requestor = srm.requestor;
	// RequestorDescription
	// -- Required objects ------------------------------------ //
	//	VehicleID
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	RequestorType                       INCL
	//	RequestorPositionVector             INCL
	//	DescriptiveName.name                EXCL
	//	DescriptiveName.routeName           EXCL
	//	TransitVehicleStatus                EXCL
	//	TransitVehicleOccupancy             EXCL
	//	DeltaTime.transitSchedule           EXCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	// VehicleID - TemporaryID
	requestor.id.present = VehicleID_PR_entityID;
	if (!id2temporaryId(&requestor.id.choice.entityID.buf, requestor.id.choice.entityID.size, srmIn.vehId))
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".VehicleID" << std::endl;
		return(false);
	}
	// RequestorType
	if ((requestor.type = (RequestorType_t *)calloc(1, sizeof(RequestorType_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".RequestorType" << std::endl;
		return(false);
	}
	// RequestorType
	// -- Required objects ------------------------------------ //
	//	BasicVehicleRole
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	RequestSubRole                      EXCL
	//	RequestImportanceLevel              EXCL
	//	Iso3833VehicleType                  EXCL
	//	VehicleType                         INCL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	// BasicVehicleRole
	requestor.type->role = static_cast<BasicVehicleRole_t>(srmIn.vehRole);
	// VehicleType
	if ((requestor.type->hpmsType = (VehicleType_t *)calloc(1, sizeof(VehicleType_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".RequestorType.VehicleType" << std::endl;
		return(false);
	}
	*(requestor.type->hpmsType) = static_cast<VehicleType_t>(srmIn.vehType);
	// RequestorPositionVector
	if ((requestor.position = (RequestorPositionVector_t *)calloc(1, sizeof(RequestorPositionVector_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".RequestorPositionVector" << std::endl;
		return(false);
	}
	// RequestorPositionVector
	// -- Required objects ------------------------------------ //
	//	Position3D
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	heading                             INCL
	//	speed                               INCL
	// -------------------------------------------------------- //
	// Position3D
	requestor.position->position.lat  = srmIn.latitude;
	requestor.position->position.Long = srmIn.longitude;
	if (srmIn.elevation > MsgEnum::unknown_elevation)
	{
		if ((requestor.position->position.elevation = (DSRC_Elevation_t *)calloc(1, sizeof(DSRC_Elevation_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level;
			std::cerr	<< ".RequestorPositionVector.Position3D.elevation" << std::endl;
			return(false);
		}
		*(requestor.position->position.elevation) = srmIn.elevation;
	}
	// heading
	if ((requestor.position->heading = (DSRC_Angle_t *)calloc(1, sizeof(DSRC_Angle_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level;
		std::cerr	<< ".RequestorPositionVector.heading" << std::endl;
		return(false);
	}
	*(requestor.position->heading) = srmIn.heading;
	// speed
	if ((requestor.position->speed = (TransmissionAndSpeed_t *)calloc(1, sizeof(TransmissionAndSpeed_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level;
		std::cerr << ".RequestorPositionVector.speed" << std::endl;
		return(false);
	}
	requestor.position->speed->transmisson = static_cast<TransmissionState_t>(srmIn.transState);
	requestor.position->speed->speed = srmIn.speed;
	return(true);
};

/// fill SignalStatusMessage_t structure
auto ssm2msgFrame = [](const SSM_element_t& ssmIn, SignalStatusMessage_t& ssm)
{
	if (ssmIn.mpSignalRequetStatus.empty())
		return(false);  // nothing to encode
	// SSM:
	// -- Required objects ------------------------------------ //
	//	DSecond
	//	SignalStatusList
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	MinuteOfTheYear                     OPTIONAL
	//	MsgCount                            OPTIONAL
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //
	std::string prog_name{"ssm2msgFrame :"};
	std::string allocate_level{"SSM"};
	// DSecond
	ssm.second = ssmIn.timeStampSec;
	// MinuteOfTheYear
	if (ssmIn.timeStampMinute < MsgEnum::invalid_timeStampMinute)
	{
		if ((ssm.timeStamp = (MinuteOfTheYear_t *)calloc(1, sizeof(MinuteOfTheYear_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".MinuteOfTheYear" << std::endl;
			return(false);
		}
		*(ssm.timeStamp) = ssmIn.timeStampMinute;
	}
	// MsgCount
	if (ssmIn.msgCnt < 0xFF)
	{
		if ((ssm.sequenceNumber = (DSRC_MsgCount_t *)calloc(1, sizeof(DSRC_MsgCount_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".MsgCount" << std::endl;
			return(false);
		}
		*(ssm.sequenceNumber) = ssmIn.msgCnt;
	}
	// SignalStatusList - one intersection per SSM
	allocate_level += ".SignalStatusList";
	if ((ssm.status.list.array = (SignalStatus_t **)calloc(1, sizeof(SignalStatus_t *))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << std::endl;
		return(false);
	}
	ssm.status.list.size  = 1;
	allocate_level += ".SignalStatus";
	if ((ssm.status.list.array[0] = (SignalStatus_t *)calloc(1, sizeof(SignalStatus_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << std::endl;
		return(false);
	}
	ssm.status.list.count = 1;
	SignalStatus_t* pSignalStatus = ssm.status.list.array[0];
	// SignalStatus
	// -- Required objects ------------------------------------ //
	//	MsgCount
	//	IntersectionReferenceID
	//	SignalStatusPackageList
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	RegionalExtension                   EXCL
	// -------------------------------------------------------- //

	// MsgCount
	pSignalStatus->sequenceNumber = ssmIn.updateCnt;
	// IntersectionReferenceID
	// -- Required objects ------------------------------------ //
	//	IntersectionID
	// -- OPTIONAL objects ------------ including/excluding  -- //
	//	RoadRegulatorID                     INCL
	// -------------------------------------------------------- //
	pSignalStatus->id.id = ssmIn.id;
	if ((pSignalStatus->id.region = (RoadRegulatorID_t *)calloc(1, sizeof(RoadRegulatorID_t))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << ".id.region" << std::endl;
		return(false);
	}
	*(pSignalStatus->id.region) = ssmIn.regionalId;
	// SignalStatusPackageList
	allocate_level += ".SignalStatusPackageList";
	if ((pSignalStatus->sigStatus.list.array = (SignalStatusPackage_t **)calloc(ssmIn.mpSignalRequetStatus.size(), sizeof(SignalStatusPackage_t *))) == NULL)
	{
		std::cerr << prog_name << "failed allocate " << allocate_level << std::endl;
		return(false);
	}
	pSignalStatus->sigStatus.list.size = static_cast<int>(ssmIn.mpSignalRequetStatus.size());

	// loop through the list of priority/preemption requests
	allocate_level += ".SignalStatusPackage";
	bool has_error = false;  // for earlier return
	int& statusListCnt = pSignalStatus->sigStatus.list.count;
	for (const auto& signalRequetStatus : ssmIn.mpSignalRequetStatus)
	{
		if ((pSignalStatus->sigStatus.list.array[statusListCnt] = (SignalStatusPackage_t *)calloc(1, sizeof(SignalStatusPackage_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << std::endl;
			has_error = true;
			break;
		}
		SignalStatusPackage_t* pSignalStatusPackage = pSignalStatus->sigStatus.list.array[statusListCnt++];
		// SignalStatusPackage
		// -- Required objects ------------------------------------ //
		//	inboundOn
		//	PrioritizationResponseStatus
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	SignalRequesterInfo                 INCL
		//	outboundOn                          OPTIONAL
		//  ETA.minute                          OPTIONAL
		//  ETA.second                          OPTIONAL
		//	duration                            OPTIONAL
		//	RegionalExtension                   EXCL
		// -------------------------------------------------------- //

		// PrioritizationResponseStatus
		pSignalStatusPackage->status = static_cast<PrioritizationResponseStatus_t>(signalRequetStatus.status);
		// inboundOn
		if ((signalRequetStatus.inApprochId == 0) && (signalRequetStatus.inLaneId == 0))
		{
			std::cerr << prog_name << "either entry approach or lane needs to be specified" << std::endl;
			has_error = true;
			break;
		}
		if (signalRequetStatus.inLaneId == 0)
		{
			pSignalStatusPackage->inboundOn.present = IntersectionAccessPoint_PR_approach;
			pSignalStatusPackage->inboundOn.choice.approach = signalRequetStatus.inApprochId;
		}
		else
		{
			pSignalStatusPackage->inboundOn.present = IntersectionAccessPoint_PR_lane;
			pSignalStatusPackage->inboundOn.choice.lane = signalRequetStatus.inLaneId;
		}
		// outboundOn
		if ((signalRequetStatus.outApproachId != 0) || (signalRequetStatus.outLaneId != 0))
		{
			if ((pSignalStatusPackage->outboundOn = (IntersectionAccessPoint_t *)calloc(1, sizeof(IntersectionAccessPoint_t))) == NULL)
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << ".outboundOn" << std::endl;
				has_error = true;
				break;
			}
			if (signalRequetStatus.outLaneId == 0)
			{
				pSignalStatusPackage->outboundOn->present = IntersectionAccessPoint_PR_approach;
				pSignalStatusPackage->outboundOn->choice.approach = signalRequetStatus.outApproachId;
			}
			else
			{
				pSignalStatusPackage->outboundOn->present = IntersectionAccessPoint_PR_lane;
				pSignalStatusPackage->outboundOn->choice.lane = signalRequetStatus.outLaneId;
			}
		}
		// ETA
		if (signalRequetStatus.ETAminute < MsgEnum::invalid_timeStampMinute)
		{
			if ((pSignalStatusPackage->minute =	(MinuteOfTheYear_t *)calloc(1, sizeof(MinuteOfTheYear_t))) == NULL)
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << ".ETAminute" << std::endl;
				has_error = true;
				break;
			}
			*(pSignalStatusPackage->minute) = signalRequetStatus.ETAminute;
		}
		if (signalRequetStatus.ETAsec < 0xFFFF)
		{
			if ((pSignalStatusPackage->second = (DSecond_t *)calloc(1, sizeof(DSecond_t))) == NULL)
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << ".ETAsec" << std::endl;
				has_error = true;
				break;
			}
			*(pSignalStatusPackage->second) = signalRequetStatus.ETAsec;
		}
		// duration
		if (signalRequetStatus.duration < 0xFFFF)
		{
			if ((pSignalStatusPackage->duration = (DSecond_t *)calloc(1, sizeof(DSecond_t))) == NULL)
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << ".duration" << std::endl;
				has_error = true;
				break;
			}
			*(pSignalStatusPackage->duration) = signalRequetStatus.duration;
		}
		// SignalRequesterInfo
		if ((pSignalStatusPackage->requester = (SignalRequesterInfo_t *)calloc(1, sizeof(SignalRequesterInfo_t))) == NULL)
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".SignalRequesterInfo" << std::endl;
			has_error = true;
			break;
		}
		SignalRequesterInfo_t* pSignalRequesterInfo = pSignalStatusPackage->requester;
		// SignalRequesterInfo
		// -- Required objects ------------------------------------ //
		//	VehicleID
		//	RequestID
		//	MsgCount
		// -- OPTIONAL objects ------------ including/excluding  -- //
		//	BasicVehicleRole                    OPTIONAL
		//	RequestorType                       EXCL
		// -------------------------------------------------------- //

		// VehicleID - TemporaryID
		pSignalRequesterInfo->id.present = VehicleID_PR_entityID;
		if (!id2temporaryId(&(pSignalRequesterInfo->id.choice.entityID.buf), pSignalRequesterInfo->id.choice.entityID.size, signalRequetStatus.vehId))
		{
			std::cerr << prog_name << "failed allocate " << allocate_level << ".SignalRequesterInfo.VehicleID" << std::endl;
			has_error = true;
			break;
		}
		// RequestID
		pSignalRequesterInfo->request = signalRequetStatus.reqId;
		// MsgCount
		pSignalRequesterInfo->sequenceNumber = signalRequetStatus.sequenceNumber;
		// BasicVehicleRole
		if (signalRequetStatus.vehRole != MsgEnum::basicRole::unavailable)
		{
			if ((pSignalRequesterInfo->role = (BasicVehicleRole_t *)calloc(1, sizeof(BasicVehicleRole_t))) == NULL)
			{
				std::cerr << prog_name << "failed allocate " << allocate_level << ".SignalRequesterInfo.BasicVehicleRole" << std::endl;
				has_error = true;
				break;
			}
			*(pSignalRequesterInfo->role) = static_cast<BasicVehicleRole_t>(signalRequetStatus.vehRole);
		}
	}
	return(!has_error);
};

/// convert MapData_t, which specifies speed limit at lane/node level, to MapData_element_t
auto msgFrame2mapData_single = [](const MapData_t& mapData, MapData_element_t& mapDataOut)->bool
{
	std::string prog_name{"msgFrame2mapData_single: "};
	const IntersectionGeometry_t* pIntersectionGeometry = mapData.intersections->list.array[0];
	mapDataOut.id = static_cast<uint16_t>(pIntersectionGeometry->id.id);
	mapDataOut.regionalId = (pIntersectionGeometry->id.region != NULL) ? static_cast<uint16_t>(*(pIntersectionGeometry->id.region)) : 0;
	mapDataOut.mapVersion = static_cast<uint8_t>(mapData.msgIssueRevision);
	mapDataOut.mpApproaches.resize(15);  // maximum approaches
	mapDataOut.attributes.set(1);        // Geometric data is included
	const auto& position3D = pIntersectionGeometry->refPoint;
	mapDataOut.geoRef.latitude = static_cast<int32_t>(position3D.lat);
	mapDataOut.geoRef.longitude = static_cast<int32_t>(position3D.Long);
	if ((position3D.elevation != NULL) && (*(position3D.elevation) != (DSRC_Elevation_t)MsgEnum::unknown_elevation))
	{ // Elevation data is included
		mapDataOut.attributes.set(0);
		mapDataOut.geoRef.elevation = static_cast<int32_t>(*(position3D.elevation));
	}
	else
		mapDataOut.geoRef.elevation = 0;
	// LaneWidth
	if (pIntersectionGeometry->laneWidth == NULL)
	{
		std::cerr << prog_name << "missing LaneWidth" << std::endl;
		mapDataOut.reset();
		return(false);
	}
	uint16_t refLaneWidth = static_cast<uint16_t>(*(pIntersectionGeometry->laneWidth));
	// SpeedLimitList
	uint16_t refSpeedLimt = ((pIntersectionGeometry->speedLimits != NULL)
		&& (pIntersectionGeometry->speedLimits->list.array != NULL)
		&& (pIntersectionGeometry->speedLimits->list.array[0] != NULL)
		&& (pIntersectionGeometry->speedLimits->list.array[0]->type == SpeedLimitType_vehicleMaxSpeed)) ?
		static_cast<uint16_t>(pIntersectionGeometry->speedLimits->list.array[0]->speed) : MsgEnum::unknown_speed;
	// LaneList
	if (pIntersectionGeometry->laneSet.list.count == 0)
	{
		std::cerr << prog_name << "missing LaneList" << std::endl;
		mapDataOut.reset();
		return(false);
	}
	// loop through lanes
	bool has_error = false;
	for (int j = 0; j < pIntersectionGeometry->laneSet.list.count; j++)
	{
		const GenericLane_t* pGenericLane = pIntersectionGeometry->laneSet.list.array[j];
		const auto& laneDirection = pGenericLane->laneAttributes.directionalUse;
		uint8_t directionalUse = static_cast<uint8_t>(bitString2ul(laneDirection.buf, laneDirection.size, laneDirection.bits_unused));
		if ((directionalUse < 0x01) || (directionalUse > 0x03))
		{
			std::cerr << prog_name << "laneId=" << pGenericLane->laneID;
			std::cerr << ", unknown directionalUse=" << static_cast<unsigned int>(directionalUse);
			std::cerr << ", bitString size=" << laneDirection.size;
			std::cerr << ", bits_unused=" << laneDirection.bits_unused << std::endl;
			has_error = true;
			break;
		}
		if ((pGenericLane->ingressApproach == NULL) && (pGenericLane->egressApproach == NULL))
		{
			std::cerr << prog_name << "laneId=" << pGenericLane->laneID << ", missing ApproachID" << std::endl;
			has_error = true;
			break;
		}
		if ((directionalUse == 0x01)  // inbound traffic lane
			&& ((pGenericLane->connectsTo == NULL) || (pGenericLane->connectsTo->list.count == 0)
				|| (pGenericLane->connectsTo->list.array[0]->signalGroup == NULL)))
		{ // required to provide signal phase that controls the movement
			std::cerr << prog_name << "laneId=" << pGenericLane->laneID << ", missing ConnectsTo" << std::endl;
			has_error = true;
			break;
		}
		if ((pGenericLane->nodeList.present != NodeListXY_PR_nodes)
			|| (pGenericLane->nodeList.choice.nodes.list.array == NULL)
			|| (pGenericLane->nodeList.choice.nodes.list.count < 2)
			|| (pGenericLane->nodeList.choice.nodes.list.count > 63))
		{
			std::cerr << prog_name << "laneId=" << pGenericLane->laneID << ", missing NodeListXY" << std::endl;
			has_error = true;
			break;
		}
		uint8_t approachId = static_cast<uint8_t>((pGenericLane->ingressApproach != NULL) ?
			(*(pGenericLane->ingressApproach)) : (*(pGenericLane->egressApproach)));
		if ((approachId == 0) || (approachId > 15))
		{
			std::cerr << prog_name << "laneId=" << pGenericLane->laneID << ", out-of-bound approachId ";
			std::cerr	<< static_cast<unsigned int>(approachId) << std::endl;
			has_error = true;
			break;
		}
		auto& appStruct = mapDataOut.mpApproaches[approachId-1];
		if (appStruct.id != approachId)
		{ // assign ApproachStruct variables
			appStruct.id = approachId;
			appStruct.speed_limit = (directionalUse == 0x03) ? 0 : MsgEnum::unknown_speed;
			switch(directionalUse)
			{
			case 0x01:
				appStruct.type = MsgEnum::approachType::inbound;
				break;
			case 0x02:
				appStruct.type = MsgEnum::approachType::outbound;
				break;
			case 0x03:
				appStruct.type = MsgEnum::approachType::crosswalk;
				break;
			}
		}
		// assign LaneStruct variables
		lane_element_t laneStruct;
		laneStruct.id = static_cast<uint8_t>(pGenericLane->laneID);
		laneStruct.type = (directionalUse == 0x03) ? (MsgEnum::laneType::crosswalk) : (MsgEnum::laneType::traffic);
		const auto& laneType = pGenericLane->laneAttributes.laneType;
		unsigned long laneTypeAttrib = (laneType.present == LaneTypeAttributes_PR_crosswalk)
			? bitString2ul(laneType.choice.crosswalk.buf, laneType.choice.crosswalk.size, laneType.choice.crosswalk.bits_unused)
			: bitString2ul(laneType.choice.vehicle.buf, laneType.choice.vehicle.size, laneType.choice.vehicle.bits_unused);
		unsigned long allowedManeuvers = 0;
		if (directionalUse != 0x03)
		{ // not crosswalk
			if (pGenericLane->maneuvers == NULL)
				std::cerr << prog_name << "laneId=" << pGenericLane->laneID << ", missing AllowedManeuvers" << std::endl;
			else
				allowedManeuvers = bitString2ul(pGenericLane->maneuvers->buf, pGenericLane->maneuvers->size, pGenericLane->maneuvers->bits_unused);
		}
		laneStruct.attributes = std::bitset<20>(allowedManeuvers << 8 | laneTypeAttrib);
		laneStruct.controlPhase = ((pGenericLane->connectsTo == NULL) || (pGenericLane->connectsTo->list.count == 0)
			|| (pGenericLane->connectsTo->list.array[0]->signalGroup == NULL)) ? 0
			: static_cast<uint8_t>(*(pGenericLane->connectsTo->list.array[0]->signalGroup));
		// loop through connecting lanes
		if ((pGenericLane->connectsTo != NULL) && (pGenericLane->connectsTo->list.count > 0))
		{
			laneStruct.mpConnectTo.resize(pGenericLane->connectsTo->list.count);
			for (int k = 0; k < pGenericLane->connectsTo->list.count; k++)
			{ // assign ConnectStruct variables
				const Connection_t* pConnection = pGenericLane->connectsTo->list.array[k];
				const auto& connectingLane = pConnection->connectingLane;
				laneStruct.mpConnectTo[k].intersectionId = (pConnection->remoteIntersection == NULL) ?
					mapDataOut.id : static_cast<uint16_t>(pConnection->remoteIntersection->id);
				laneStruct.mpConnectTo[k].regionalId = ((pConnection->remoteIntersection == NULL) || (pConnection->remoteIntersection->region == NULL)) ?
					mapDataOut.regionalId : static_cast<uint16_t>(*(pConnection->remoteIntersection->region));
				laneStruct.mpConnectTo[k].laneId = static_cast<uint8_t>(connectingLane.lane);
				unsigned long connecting_maneuvers = (connectingLane.maneuver == NULL) ? 0 :
					bitString2ul(connectingLane.maneuver->buf, connectingLane.maneuver->size, connectingLane.maneuver->bits_unused);
				switch(connecting_maneuvers)
				{
				case 0x00:
					laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::unavailable;
					break;
				case 0x01:
					laneStruct.mpConnectTo[k].laneManeuver = (directionalUse == 0x01) ?
						(MsgEnum::maneuverType::straightAhead) : (MsgEnum::maneuverType::straight);
					break;
				case 0x02:
					laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::leftTurn;
					break;
				case 0x04:
					laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::rightTurn;
					break;
				case 0x08:
					laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::uTurn;
					break;
				}
			}
		}
		// loop through lane nodes
		laneStruct.mpNodes.resize(pGenericLane->nodeList.choice.nodes.list.count);
		int nodeCnt = 0;
		for (int k = 0; k < pGenericLane->nodeList.choice.nodes.list.count; k++)
		{
			const NodeXY_t* pNode = pGenericLane->nodeList.choice.nodes.list.array[k];
			int32_t& offset_x = laneStruct.mpNodes[nodeCnt].offset_x;
			int32_t& offset_y = laneStruct.mpNodes[nodeCnt].offset_y;
			int32_t& latitude = laneStruct.mpNodes[nodeCnt].latitude;
			int32_t& longitude = laneStruct.mpNodes[nodeCnt].longitude;
			switch(pNode->delta.present)
			{
			case NodeOffsetPointXY_PR_node_XY1:
				offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY1.x);
				offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY1.y);
				laneStruct.mpNodes[nodeCnt].useXY = true;
				break;
			case NodeOffsetPointXY_PR_node_XY2:
				offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY2.x);
				offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY2.y);
				laneStruct.mpNodes[nodeCnt].useXY = true;
				break;
			case NodeOffsetPointXY_PR_node_XY3:
				offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY3.x);
				offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY3.y);
				laneStruct.mpNodes[nodeCnt].useXY = true;
				break;
			case NodeOffsetPointXY_PR_node_XY4:
				offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY4.x);
				offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY4.y);
				laneStruct.mpNodes[nodeCnt].useXY = true;
				break;
			case NodeOffsetPointXY_PR_node_XY5:
				offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY5.x);
				offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY5.y);
				laneStruct.mpNodes[nodeCnt].useXY = true;
				break;
			case NodeOffsetPointXY_PR_node_XY6:
				offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY6.x);
				offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY6.y);
				laneStruct.mpNodes[nodeCnt].useXY = true;
				break;
			case NodeOffsetPointXY_PR_node_LatLon:
				latitude  = static_cast<int32_t>(pNode->delta.choice.node_LatLon.lat);
				longitude = static_cast<int32_t>(pNode->delta.choice.node_LatLon.lon);
				laneStruct.mpNodes[nodeCnt].useXY = false;
				break;
			default:
				continue;
				break;
			}
			if ((k == 0) && (pNode->attributes != NULL))
			{
				if (pNode->attributes->dWidth != NULL)
					refLaneWidth = static_cast<uint16_t>(refLaneWidth + *(pNode->attributes->dWidth));
				if ((directionalUse != 0x03) && (pNode->attributes->data != NULL) && (pNode->attributes->data->list.array != NULL)
					&& (pNode->attributes->data->list.array[0] != NULL)
					&& (pNode->attributes->data->list.array[0]->present == LaneDataAttribute_PR_speedLimits)
					&& (pNode->attributes->data->list.array[0]->choice.speedLimits.list.array != NULL)
					&& (pNode->attributes->data->list.array[0]->choice.speedLimits.list.array[0] != NULL)
					&& (pNode->attributes->data->list.array[0]->choice.speedLimits.list.array[0]->type == SpeedLimitType_vehicleMaxSpeed))
				{
					refSpeedLimt = static_cast<uint16_t>(pNode->attributes->data->list.array[0]->choice.speedLimits.list.array[0]->speed);
				}
			}
			nodeCnt++;
		}
		laneStruct.width = refLaneWidth;
		if ((directionalUse != 0x03) && (appStruct.speed_limit != refSpeedLimt))
			appStruct.speed_limit = refSpeedLimt;
		if (std::find(mapDataOut.speeds.begin(), mapDataOut.speeds.end(),	appStruct.speed_limit) == mapDataOut.speeds.end())
			mapDataOut.speeds.push_back(appStruct.speed_limit);
		if (nodeCnt != pGenericLane->nodeList.choice.nodes.list.count)
			laneStruct.mpNodes.resize(nodeCnt);
		appStruct.mpLanes.push_back(laneStruct);
	}
	if (std::count_if(mapDataOut.speeds.begin(), mapDataOut.speeds.end(),
		[](const uint16_t& item){return((item > 0) && (item < MsgEnum::unknown_speed));}) == 0)
	{
		std::cerr << prog_name << "missing speedLimit" << std::endl;
		has_error = true;
	}
	else
		mapDataOut.attributes.set(2);
	if (has_error)
		mapDataOut.reset();
	return(!has_error);
};

/// convert MapData_t, which specifies speed limit at the approach level, to MapData_element_t
auto msgFrame2mapData_multi = [](const MapData_t& mapData, MapData_element_t& mapDataOut)->bool
{
	std::string prog_name{"msgFrame2mapData_multi: "};
	mapDataOut.id = static_cast<uint16_t>(mapData.intersections->list.array[0]->id.id);
	if (mapData.intersections->list.array[0]->id.region != NULL)
		mapDataOut.regionalId = static_cast<uint16_t>(*(mapData.intersections->list.array[0]->id.region));
	mapDataOut.mapVersion = static_cast<uint8_t>(mapData.msgIssueRevision);
	mapDataOut.mpApproaches.resize(15);  // maximum approaches
	mapDataOut.attributes.set(1);        // Geometric data is included
	const auto& position3D = mapData.intersections->list.array[0]->refPoint;
	mapDataOut.geoRef.latitude = static_cast<int32_t>(position3D.lat);
	mapDataOut.geoRef.longitude = static_cast<int32_t>(position3D.Long);
	if ((position3D.elevation != NULL) && (*(position3D.elevation) != (DSRC_Elevation_t)MsgEnum::unknown_elevation))
	{ // Elevation data is included
		mapDataOut.attributes.set(0);
		mapDataOut.geoRef.elevation = static_cast<int32_t>(*(position3D.elevation));
	}
	else
		mapDataOut.geoRef.elevation = 0;
	// loop through speed groups
	bool has_error = false;
	for (int i = 0; i < mapData.intersections->list.count; i++)
	{
		const IntersectionGeometry_t* pIntersectionGeometry = mapData.intersections->list.array[i];
		if (static_cast<uint16_t>(pIntersectionGeometry->id.id) != mapDataOut.id)
		{
			std::cerr << prog_name << "containing multiples IntersectionReferenceID" << std::endl;
			has_error = true;
			break;
		}
		if (pIntersectionGeometry->laneWidth == NULL)
		{
			std::cerr << prog_name << "missing LaneWidth" << std::endl;
			has_error = true;
			break;
		}
		uint16_t refLaneWidth = static_cast<uint16_t>(*(pIntersectionGeometry->laneWidth));
		if (pIntersectionGeometry->laneSet.list.count == 0)
		{
			std::cerr << prog_name << "missing LaneList" << std::endl;
			has_error = true;
			break;
		}
		// speed limit for this speed group
		const auto& firstLaneDirection = pIntersectionGeometry->laneSet.list.array[0]->laneAttributes.directionalUse;
		uint8_t  laneDirectionalUse = static_cast<uint8_t>(bitString2ul(firstLaneDirection.buf, firstLaneDirection.size, firstLaneDirection.bits_unused));
		if ((laneDirectionalUse < 0x01) || (laneDirectionalUse > 0x03))
		{
			std::cerr << prog_name << "unknown laneDirectionalUse=" << static_cast<unsigned int>(laneDirectionalUse);
			std::cerr << ", bitString size=" << firstLaneDirection.size;
			std::cerr << ", bits_unused=" << firstLaneDirection.bits_unused << std::endl;
			has_error = true;
			break;
		}
		uint16_t speed_limit = (laneDirectionalUse == 0x03) ? 0 : MsgEnum::unknown_speed;
		// check whether or not the speed limit is set for vehicular traffic lanes
		if ((laneDirectionalUse != 0x03) && (pIntersectionGeometry->speedLimits != NULL))
		{ // loop through speedLimits to find SpeedLimitType_vehicleMaxSpeed entry
			for (int j = 0; j < pIntersectionGeometry->speedLimits->list.count; j++)
			{
				const RegulatorySpeedLimit_t* pRegulatorySpeedLimit = pIntersectionGeometry->speedLimits->list.array[j];
				if ((pRegulatorySpeedLimit->type == SpeedLimitType_vehicleMaxSpeed)
					&& (pRegulatorySpeedLimit->speed != (Velocity_t)MsgEnum::unknown_speed))
				{ // speed limit data is included
					speed_limit = static_cast<uint16_t>(pRegulatorySpeedLimit->speed);
					break;
				}
			}
		}
		// loop through lanes
		for (int j = 0; j < pIntersectionGeometry->laneSet.list.count; j++)
		{
			const GenericLane_t* pGenericLane = pIntersectionGeometry->laneSet.list.array[j];
			const auto& laneDirection = pGenericLane->laneAttributes.directionalUse;
			uint8_t directionalUse = static_cast<uint8_t>(bitString2ul(laneDirection.buf, laneDirection.size, laneDirection.bits_unused));
			if ((directionalUse < 0x01) || (directionalUse > 0x03))
			{
				std::cerr << prog_name << "laneId=" << pGenericLane->laneID;
				std::cerr << ", unknown directionalUse=" << static_cast<unsigned int>(directionalUse);
				std::cerr << ", bitString size=" << laneDirection.size;
				std::cerr << ", bits_unused=" << laneDirection.bits_unused << std::endl;
				has_error = true;
				break;
			}
			if ((pGenericLane->ingressApproach == NULL) && (pGenericLane->egressApproach == NULL))
			{
				std::cerr << prog_name << "laneId=" << pGenericLane->laneID << ", missing ApproachID" << std::endl;
				has_error = true;
				break;
			}
			if ((directionalUse == 0x01)  // inbound traffic lane
				&& ((pGenericLane->connectsTo == NULL) || (pGenericLane->connectsTo->list.count == 0)
					|| (pGenericLane->connectsTo->list.array[0]->signalGroup == NULL)))
			{ // required to provide signal phase that controls the movement
				std::cerr << prog_name << "laneId=" << pGenericLane->laneID << ", missing ConnectsTo" << std::endl;
				has_error = true;
				break;
			}
			if ((pGenericLane->nodeList.present != NodeListXY_PR_nodes)
				|| (pGenericLane->nodeList.choice.nodes.list.array == NULL)
				|| (pGenericLane->nodeList.choice.nodes.list.count < 2)
				|| (pGenericLane->nodeList.choice.nodes.list.count > 63))
			{
				std::cerr << prog_name << "laneId=" << pGenericLane->laneID << ", missing NodeListXY" << std::endl;
				has_error = true;
				break;
			}
			uint8_t approachId = static_cast<uint8_t>((pGenericLane->ingressApproach != NULL) ?
				(*(pGenericLane->ingressApproach)) : (*(pGenericLane->egressApproach)));
			if ((approachId > 0) && (approachId <= 15) && (mapDataOut.mpApproaches[approachId-1].id != approachId))
			{ // assign ApproachStruct variables
				mapDataOut.mpApproaches[approachId-1].id = approachId;
				mapDataOut.mpApproaches[approachId-1].speed_limit = speed_limit;
				if (std::find(mapDataOut.speeds.begin(), mapDataOut.speeds.end(),	speed_limit) == mapDataOut.speeds.end())
					mapDataOut.speeds.push_back(speed_limit);
				switch(directionalUse)
				{
				case 0x01:
					mapDataOut.mpApproaches[approachId-1].type = MsgEnum::approachType::inbound;
					break;
				case 0x02:
					mapDataOut.mpApproaches[approachId-1].type = MsgEnum::approachType::outbound;
					break;
				case 0x03:
					mapDataOut.mpApproaches[approachId-1].type = MsgEnum::approachType::crosswalk;
					break;
				}
			}
			// assign LaneStruct variables
			lane_element_t laneStruct;
			laneStruct.id = static_cast<uint8_t>(pGenericLane->laneID);
			laneStruct.type = (directionalUse == 0x03) ? (MsgEnum::laneType::crosswalk) : (MsgEnum::laneType::traffic);
			const auto& laneType = pGenericLane->laneAttributes.laneType;
			unsigned long laneTypeAttrib = (laneType.present == LaneTypeAttributes_PR_crosswalk)
				? bitString2ul(laneType.choice.crosswalk.buf, laneType.choice.crosswalk.size, laneType.choice.crosswalk.bits_unused)
				: bitString2ul(laneType.choice.vehicle.buf, laneType.choice.vehicle.size, laneType.choice.vehicle.bits_unused);
			unsigned long allowedManeuvers = 0;
			if (directionalUse != 0x03)
			{ // not crosswalk
				if (pGenericLane->maneuvers == NULL)
					std::cerr << prog_name << "laneId=" << pGenericLane->laneID << ", missing AllowedManeuvers" << std::endl;
				else
					allowedManeuvers = bitString2ul(pGenericLane->maneuvers->buf, pGenericLane->maneuvers->size, pGenericLane->maneuvers->bits_unused);
			}
			laneStruct.attributes = std::bitset<20>(allowedManeuvers << 8 | laneTypeAttrib);
			const NodeXY_t* pFirstNode = pGenericLane->nodeList.choice.nodes.list.array[0];
			laneStruct.width = ((pFirstNode->attributes != NULL) && (pFirstNode->attributes->dWidth != NULL))
				? (uint16_t)(refLaneWidth + *(pFirstNode->attributes->dWidth)) : refLaneWidth;
			laneStruct.controlPhase = ((pGenericLane->connectsTo == NULL) || (pGenericLane->connectsTo->list.count == 0)
				|| (pGenericLane->connectsTo->list.array[0]->signalGroup == NULL)) ? 0
				: static_cast<uint8_t>(*(pGenericLane->connectsTo->list.array[0]->signalGroup));
			// loop through connecting lanes
			if (pGenericLane->connectsTo != NULL)
			{
				laneStruct.mpConnectTo.resize(pGenericLane->connectsTo->list.count);
				for (int k = 0; k < pGenericLane->connectsTo->list.count; k++)
				{ // assign ConnectStruct variables
					const Connection_t* pConnection = pGenericLane->connectsTo->list.array[k];
					const auto& connectingLane = pConnection->connectingLane;
					laneStruct.mpConnectTo[k].intersectionId = (pConnection->remoteIntersection == NULL) ?
						mapDataOut.id : static_cast<uint16_t>(pConnection->remoteIntersection->id);
					laneStruct.mpConnectTo[k].regionalId = ((pConnection->remoteIntersection == NULL) || (pConnection->remoteIntersection->region == NULL)) ?
						mapDataOut.regionalId : static_cast<uint16_t>(*(pConnection->remoteIntersection->region));
					laneStruct.mpConnectTo[k].laneId = static_cast<uint8_t>(connectingLane.lane);
					unsigned long connecting_maneuvers = (connectingLane.maneuver == NULL) ? 0 :
						bitString2ul(connectingLane.maneuver->buf, connectingLane.maneuver->size, connectingLane.maneuver->bits_unused);
					switch(connecting_maneuvers)
					{
					case 0x00:
						laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::unavailable;
						break;
					case 0x01:
						laneStruct.mpConnectTo[k].laneManeuver = (directionalUse == 0x01) ?
							(MsgEnum::maneuverType::straightAhead) : (MsgEnum::maneuverType::straight);
						break;
					case 0x02:
						laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::leftTurn;
						break;
					case 0x04:
						laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::rightTurn;
						break;
					case 0x08:
						laneStruct.mpConnectTo[k].laneManeuver = MsgEnum::maneuverType::uTurn;
						break;
					}
				}
			}
			// loop through lane nodes
			laneStruct.mpNodes.resize(pGenericLane->nodeList.choice.nodes.list.count);
			int nodeCnt = 0;
			for (int k = 0; k < pGenericLane->nodeList.choice.nodes.list.count; k++)
			{
				const NodeXY_t* pNode = pGenericLane->nodeList.choice.nodes.list.array[k];
				int32_t& offset_x = laneStruct.mpNodes[nodeCnt].offset_x;
				int32_t& offset_y = laneStruct.mpNodes[nodeCnt].offset_y;
				int32_t& latitude = laneStruct.mpNodes[nodeCnt].latitude;
				int32_t& longitude = laneStruct.mpNodes[nodeCnt].longitude;
				switch(pNode->delta.present)
				{
				case NodeOffsetPointXY_PR_node_XY1:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY1.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY1.y);
					laneStruct.mpNodes[nodeCnt].useXY = true;
					break;
				case NodeOffsetPointXY_PR_node_XY2:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY2.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY2.y);
					laneStruct.mpNodes[nodeCnt].useXY = true;
					break;
				case NodeOffsetPointXY_PR_node_XY3:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY3.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY3.y);
					laneStruct.mpNodes[nodeCnt].useXY = true;
					break;
				case NodeOffsetPointXY_PR_node_XY4:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY4.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY4.y);
					laneStruct.mpNodes[nodeCnt].useXY = true;
					break;
				case NodeOffsetPointXY_PR_node_XY5:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY5.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY5.y);
					laneStruct.mpNodes[nodeCnt].useXY = true;
					break;
				case NodeOffsetPointXY_PR_node_XY6:
					offset_x = static_cast<int32_t>(pNode->delta.choice.node_XY6.x);
					offset_y = static_cast<int32_t>(pNode->delta.choice.node_XY6.y);
					laneStruct.mpNodes[nodeCnt].useXY = true;
					break;
				case NodeOffsetPointXY_PR_node_LatLon:
					latitude  = static_cast<int32_t>(pNode->delta.choice.node_LatLon.lat);
					longitude = static_cast<int32_t>(pNode->delta.choice.node_LatLon.lon);
					laneStruct.mpNodes[nodeCnt].useXY = false;
					break;
				default:
					continue;
					break;
				}
				nodeCnt++;
			}
			if (nodeCnt != pGenericLane->nodeList.choice.nodes.list.count)
				laneStruct.mpNodes.resize(nodeCnt);
			mapDataOut.mpApproaches[approachId-1].mpLanes.push_back(laneStruct);
		}
		if (has_error)
			break;
	}
	if (std::count_if(mapDataOut.speeds.begin(), mapDataOut.speeds.end(),
		[](const uint16_t& item){return((item > 0) && (item < MsgEnum::unknown_speed));}) == 0)
	{
		std::cerr << prog_name << "missing speedLimit" << std::endl;
		has_error = true;
	}
	else
		mapDataOut.attributes.set(2);
	if (has_error)
		mapDataOut.reset();
	return(!has_error);
};

/// convert MapData_t to MapData_element_t
auto msgFrame2mapData = [](const MapData_t& mapData, MapData_element_t& mapDataOut)->bool
{
	if ((mapData.intersections == NULL) || (mapData.intersections->list.array == NULL)
		|| (mapData.intersections->list.array[0] == NULL) || (mapData.intersections->list.count == 0))
	{
		std::cerr << "msgFrame2mapData: empty IntersectionGeometryList" << std::endl;
		return(false);
	}
	mapDataOut.isSingleFrame = (mapData.intersections->list.count == 1) ? true : false;
	return((mapDataOut.isSingleFrame) ? msgFrame2mapData_single(mapData, mapDataOut) : msgFrame2mapData_multi(mapData, mapDataOut));
};

/// convert SPAT_t to SPAT_element_t
auto msgFrame2spat = [](const SPAT_t& spat, SPAT_element_t& spatOut)->bool
{
	std::string prog_name{"msgFrame2spat: "};
	if (spat.intersections.list.count == 0)
	{
		std::cerr << prog_name << "empty IntersectionStateList" << std::endl;
		return(false);
	}
	const IntersectionState_t* pIntsectionState = spat.intersections.list.array[0];
	spatOut.id = static_cast<uint16_t>(pIntsectionState->id.id);
	if (pIntsectionState->id.region != NULL)
		spatOut.regionalId = static_cast<uint16_t>(*(pIntsectionState->id.region));
	spatOut.msgCnt = static_cast<uint8_t>(pIntsectionState->revision);
	if (pIntsectionState->moy != NULL)
		spatOut.timeStampMinute = static_cast<uint32_t>(*(pIntsectionState->moy));
	if (pIntsectionState->timeStamp != NULL)
		spatOut.timeStampSec = static_cast<uint16_t>(*(pIntsectionState->timeStamp));
	spatOut.status = std::bitset<16>(bitString2ul(pIntsectionState->status.buf, pIntsectionState->status.size, pIntsectionState->status.bits_unused));
	if (pIntsectionState->states.list.count == 0)
	{
		std::cerr << prog_name << "empty MovementList" << std::endl;
		spatOut.reset();
		return(false);
	}
	auto& permittedPhases = spatOut.permittedPhases;
	auto& permittedPedPhases = spatOut.permittedPedPhases;
	bool has_error = false;
	for (int i = 0; i < pIntsectionState->states.list.count; i++)
	{
		const MovementState_t* pMovementState = pIntsectionState->states.list.array[i];
		if ((pMovementState->signalGroup < 1) || (pMovementState->signalGroup > 2 * 8))
		{
			std::cerr << prog_name << "invalid SignalGroupID" << std::endl;
			has_error = true;
			break;
		}
		if (pMovementState->state_time_speed.list.count == 0)
			continue;
		const MovementEvent_t* pMovementEvent = pMovementState->state_time_speed.list.array[0];
		int j = static_cast<int>((pMovementState->signalGroup - 1) % 8);
		PhaseState_element_t& phaseState = (pMovementState->signalGroup > 8) ?
			spatOut.pedPhaseState[j] : spatOut.phaseState[j];
		if (pMovementState->signalGroup > 8)
			permittedPedPhases.set(j);
		else
			permittedPhases.set(j);
		phaseState.currState = static_cast<MsgEnum::phaseState>(pMovementEvent->eventState);
		if (pMovementEvent->timing != NULL)
		{
			phaseState.minEndTime = static_cast<uint16_t>(pMovementEvent->timing->minEndTime);
			if (pMovementEvent->timing->startTime != NULL)
				phaseState.startTime = static_cast<uint16_t>(*(pMovementEvent->timing->startTime));
			if (pMovementEvent->timing->maxEndTime != NULL)
				phaseState.maxEndTime = static_cast<uint16_t>(*(pMovementEvent->timing->maxEndTime));
		}
	}
	if (has_error)
		spatOut.reset();
	return(!has_error);
};

/// convert RTCMcorrections_t to RTCM_element_t
auto msgFrame2rtcm = [](const RTCMcorrections_t& rtcm, RTCM_element_t& rtcmOut)->bool
{
	std::string prog_name{"msgFrame2rtcm: "};
	if (rtcm.msgs.list.count == 0)
	{
		std::cerr << prog_name << "empty RTCMmessageList" << std::endl;
		return(false);
	}
	rtcmOut.msgCnt = static_cast<uint8_t>(rtcm.msgCnt);
	rtcmOut.rev = static_cast<uint8_t>(rtcm.rev);
	if (rtcm.timeStamp != NULL)
		rtcmOut.timeStampMinute = static_cast<uint32_t>(*(rtcm.timeStamp));
	const RTCMmessage_t* pRTCMmessage = rtcm.msgs.list.array[0];
	if (pRTCMmessage->size == 0)
	{
		std::cerr << prog_name << "empty RTCMmessage" << std::endl;
		rtcmOut.reset();
		return(false);
	}
	rtcmOut.payload.assign(pRTCMmessage->buf, pRTCMmessage->buf + pRTCMmessage->size);
	return(true);
};

/// convert SignalRequestMessage_t to SRM_element_t
auto msgFrame2srm = [](const SignalRequestMessage_t& srm, SRM_element_t& srmOut)->bool
{
	std::string prog_name{"msgFrame2srm: "};
	if ((srm.requests == NULL) || (srm.requests->list.count == 0))
	{
		std::cerr << prog_name << "missing SignalRequestList" << std::endl;
		return(false);
	}
	srmOut.timeStampSec = static_cast<uint16_t>(srm.second);
	if (srm.timeStamp != NULL)
		srmOut.timeStampMinute = static_cast<uint32_t>(*(srm.timeStamp));
	if (srm.sequenceNumber != NULL)
		srmOut.msgCnt = static_cast<uint8_t>(*(srm.sequenceNumber));
	const SignalRequestPackage_t* pSignalRequestPackage = srm.requests->list.array[0];
	if (pSignalRequestPackage->minute != NULL)
		srmOut.ETAminute = static_cast<uint32_t>(*(pSignalRequestPackage->minute));
	if (pSignalRequestPackage->second != NULL)
		srmOut.ETAsec = static_cast<uint16_t>(*(pSignalRequestPackage->second));
	if (pSignalRequestPackage->duration != NULL)
		srmOut.duration = static_cast<uint16_t>(*(pSignalRequestPackage->duration));
	const SignalRequest_t& signalRequest = pSignalRequestPackage->request;
	srmOut.intId = static_cast<uint16_t>(signalRequest.id.id);
	if (signalRequest.id.region != NULL)
		srmOut.regionalId = static_cast<uint16_t>(*(signalRequest.id.region));
	srmOut.reqId = static_cast<uint8_t>(signalRequest.requestID);
	srmOut.reqType = static_cast<MsgEnum::requestType>(signalRequest.requestType);
	if (signalRequest.inBoundLane.present == IntersectionAccessPoint_PR_approach)
		srmOut.inApprochId = static_cast<uint8_t>(signalRequest.inBoundLane.choice.approach);
	else if (signalRequest.inBoundLane.present == IntersectionAccessPoint_PR_lane)
		srmOut.inLaneId = static_cast<uint8_t>(signalRequest.inBoundLane.choice.lane);
	if (signalRequest.outBoundLane != NULL)
	{
		if (signalRequest.outBoundLane->present == IntersectionAccessPoint_PR_approach)
			srmOut.outApproachId = static_cast<uint8_t>(signalRequest.outBoundLane->choice.approach);
		else if (signalRequest.outBoundLane->present == IntersectionAccessPoint_PR_lane)
			srmOut.outLaneId = static_cast<uint8_t>(signalRequest.outBoundLane->choice.lane);
	}
	const RequestorDescription_t& requestor = srm.requestor;
	if (requestor.id.present != VehicleID_PR_entityID)
	{
		std::cerr << prog_name << "missing Temporary ID" << std::endl;
		srmOut.reset();
		return(false);
	}
	srmOut.vehId = static_cast<uint32_t>(octString2ul(requestor.id.choice.entityID.buf, requestor.id.choice.entityID.size));
	if (requestor.type == NULL)
	{
		std::cerr << prog_name << "missing RequestorType" << std::endl;
		srmOut.reset();
		return(false);
	}
	srmOut.vehRole = static_cast<MsgEnum::basicRole>(requestor.type->role);
	if (requestor.type->hpmsType == NULL)
	{
		std::cerr << prog_name << "missing VehicleType" << std::endl;
		srmOut.reset();
		return(false);
	}
	srmOut.vehType = static_cast<MsgEnum::vehicleType>(*(requestor.type->hpmsType));
	if (requestor.position == NULL)
	{
		std::cerr << prog_name << "missing RequestorPositionVector" << std::endl;
		srmOut.reset();
		return(false);
	}
	srmOut.latitude = static_cast<int32_t>(requestor.position->position.lat);
	srmOut.longitude = static_cast<int32_t>(requestor.position->position.Long);
	if (requestor.position->position.elevation != NULL)
		srmOut.elevation =static_cast<int32_t>(*(requestor.position->position.elevation));
	if (requestor.position->heading == NULL)
	{
		std::cerr << prog_name << "missing heading" << std::endl;
		srmOut.reset();
		return(false);
	}
	srmOut.heading = static_cast<uint16_t>(*(requestor.position->heading));
	if (requestor.position->speed == NULL)
	{
		std::cerr << prog_name << "missing TransmissionAndSpeed" << std::endl;
		srmOut.reset();
		return(false);
	}
	srmOut.transState = static_cast<MsgEnum::transGear>(requestor.position->speed->transmisson);
	srmOut.speed = static_cast<uint16_t>(requestor.position->speed->speed);
	return(true);
};

/// convert SignalStatusMessage_t to SSM_element_t
auto msgFrame2ssm = [](const SignalStatusMessage_t& ssm, SSM_element_t& ssmOut)->bool
{
	std::string prog_name{"msgFrame2ssm: "};
	if (ssm.status.list.count == 0)
	{
		std::cerr << prog_name << "empty SignalStatusList" << std::endl;
		return(false);
	}
	ssmOut.timeStampSec = static_cast<uint16_t>(ssm.second);
	if (ssm.timeStamp != NULL)
		ssmOut.timeStampMinute = static_cast<uint32_t>(*(ssm.timeStamp));
	if (ssm.sequenceNumber != NULL)
		ssmOut.msgCnt = static_cast<uint8_t>(*(ssm.sequenceNumber));
	const SignalStatus_t* pSignalStatus = ssm.status.list.array[0];
	ssmOut.updateCnt = static_cast<uint8_t>(pSignalStatus->sequenceNumber);
	ssmOut.id = static_cast<uint16_t>(pSignalStatus->id.id);
	if (pSignalStatus->id.region != NULL)
		ssmOut.regionalId = static_cast<uint16_t>(*(pSignalStatus->id.region));
	if (pSignalStatus->sigStatus.list.count == 0)
	{
		std::cerr << prog_name << "empty SignalStatusPackageList" << std::endl;
		ssmOut.reset();
		return(false);
	}
	ssmOut.mpSignalRequetStatus.resize(pSignalStatus->sigStatus.list.count);
	bool has_error = false;
	for (int i = 0; i < pSignalStatus->sigStatus.list.count; i++)
	{
		const SignalStatusPackage_t* pSignalStatusPackage = pSignalStatus->sigStatus.list.array[i];
		SignalRequetStatus_t& signalRequetStatus = ssmOut.mpSignalRequetStatus[i];
		signalRequetStatus.reset();
		signalRequetStatus.status = static_cast<MsgEnum::requestStatus>(pSignalStatusPackage->status);
		if (pSignalStatusPackage->inboundOn.present == IntersectionAccessPoint_PR_approach)
			signalRequetStatus.inApprochId = static_cast<uint8_t>(pSignalStatusPackage->inboundOn.choice.approach);
		else if (pSignalStatusPackage->inboundOn.present == IntersectionAccessPoint_PR_lane)
			signalRequetStatus.inLaneId = static_cast<uint8_t>(pSignalStatusPackage->inboundOn.choice.lane);
		if (pSignalStatusPackage->outboundOn != NULL)
		{
			if (pSignalStatusPackage->outboundOn->present == IntersectionAccessPoint_PR_approach)
				signalRequetStatus.outApproachId = static_cast<uint8_t>(pSignalStatusPackage->outboundOn->choice.approach);
			else if (pSignalStatusPackage->outboundOn->present == IntersectionAccessPoint_PR_lane)
				signalRequetStatus.outLaneId = static_cast<uint8_t>(pSignalStatusPackage->outboundOn->choice.lane);
		}
		if (pSignalStatusPackage->minute != NULL)
			signalRequetStatus.ETAminute = static_cast<uint32_t>(*(pSignalStatusPackage->minute));
		if (pSignalStatusPackage->second != NULL)
			signalRequetStatus.ETAsec = static_cast<uint16_t>(*(pSignalStatusPackage->second));
		if (pSignalStatusPackage->duration != NULL)
			signalRequetStatus.duration = static_cast<uint16_t>(*(pSignalStatusPackage->duration));
		if (pSignalStatusPackage->requester == NULL)
		{
			std::cerr << prog_name << "missing SignalRequesterInfo" << std::endl;
			has_error = true;
			break;
		}
		if (pSignalStatusPackage->requester->id.present != VehicleID_PR_entityID)
		{
			std::cerr << prog_name << "missing Temporary ID" << std::endl;
			has_error = true;
			break;
		}
		signalRequetStatus.vehId = static_cast<uint32_t>(octString2ul(pSignalStatusPackage->requester->id.choice.entityID.buf,
			pSignalStatusPackage->requester->id.choice.entityID.size));
		signalRequetStatus.reqId = static_cast<uint8_t>(pSignalStatusPackage->requester->request);
		signalRequetStatus.sequenceNumber = static_cast<uint8_t>(pSignalStatusPackage->requester->sequenceNumber);
		signalRequetStatus.vehRole = (pSignalStatusPackage->requester->role == NULL) ?
			MsgEnum::basicRole::unavailable : static_cast<MsgEnum::basicRole>(*(pSignalStatusPackage->requester->role));
	}
	if (has_error)
		ssmOut.reset();
	return(!has_error);
};

/// convert BasicSafetyMessage_t to BSM_element_t
auto msgFrame2bsm = [](const BasicSafetyMessage_t& bsm, BSM_element_t& bsmOut)->bool
{
	const auto& coreData = bsm.coreData;
	bsmOut.msgCnt = static_cast<uint8_t>(coreData.msgCnt);
	bsmOut.id = static_cast<uint32_t>(octString2ul(coreData.id.buf, coreData.id.size));
	bsmOut.timeStampSec = static_cast<uint16_t>(coreData.secMark);
	bsmOut.latitude = static_cast<int32_t>(coreData.lat);
	bsmOut.longitude = static_cast<int32_t>(coreData.Long);
	bsmOut.elevation = static_cast<int32_t>(coreData.elev);
	bsmOut.semiMajor = static_cast<uint8_t>(coreData.accuracy.semiMajor);
	bsmOut.semiMinor = static_cast<uint8_t>(coreData.accuracy.semiMinor);
	bsmOut.orientation = static_cast<uint16_t>(coreData.accuracy.orientation);
	bsmOut.speed = static_cast<uint16_t>(coreData.speed);
	bsmOut.heading = static_cast<uint16_t>(coreData.heading);
	bsmOut.steeringAngle = static_cast<int8_t>(coreData.angle);
	bsmOut.accelLon = static_cast<int16_t>(coreData.accelSet.Long);
	bsmOut.accelLat = static_cast<int16_t>(coreData.accelSet.lat);
	bsmOut.accelVert = static_cast<int8_t>(coreData.accelSet.vert);
	bsmOut.yawRate = static_cast<int16_t>(coreData.accelSet.yaw);
	bsmOut.vehWidth = static_cast<uint16_t>(coreData.size.width);
	bsmOut.vehLen = static_cast<uint16_t>(coreData.size.length);
	bsmOut.transState	= static_cast<MsgEnum::transGear>(coreData.transmission);
	const auto& brakeSystemStatus = coreData.brakes;
	const auto& wheelBrakes = brakeSystemStatus.wheelBrakes;
	bsmOut.brakeAppliedStatus = std::bitset<5>(bitString2ul(wheelBrakes.buf, wheelBrakes.size, wheelBrakes.bits_unused));
	bsmOut.tractionControlStatus = static_cast<MsgEnum::engageStatus>(brakeSystemStatus.traction);
	bsmOut.absStatus = static_cast<MsgEnum::engageStatus>(brakeSystemStatus.abs);
	bsmOut.stabilityControlStatus = static_cast<MsgEnum::engageStatus>(brakeSystemStatus.scs);
	bsmOut.brakeBoostApplied = static_cast<MsgEnum::engageStatus>(brakeSystemStatus.brakeBoost);
	bsmOut.auxiliaryBrakeStatus = static_cast<MsgEnum::engageStatus>(brakeSystemStatus.auxBrakes);
	return(true);
};

/// SAE J2735 UPER encoding function
size_t AsnJ2735Lib::encode_msgFrame(const Frame_element_t& dsrcFrameIn, uint8_t* buf, size_t size)
{
	MessageFrame_t* pMessageFrame = (MessageFrame_t *)calloc(1, sizeof(MessageFrame_t));
	if (pMessageFrame == NULL)
	{
		std::cerr << "encode_msgFrame: failed allocate MessageFrame" << std::endl;
		return(0);
	}
	// MessageFrame:
	// -- Required objects ------------------------------------ //
	// messageId
	// value
	// -------------------------------------------------------- //
	bool tf2msgFrame;
	switch(dsrcFrameIn.dsrcMsgId)
	{
	case MsgEnum::DSRCmsgID_map:
		pMessageFrame->value.present = MessageFrame__value_PR_MapData;
		tf2msgFrame = (dsrcFrameIn.mapData.isSingleFrame)
			? mapData2msgFrame_single(dsrcFrameIn.mapData, pMessageFrame->value.choice.MapData)
			: mapData2msgFrame_multi(dsrcFrameIn.mapData, pMessageFrame->value.choice.MapData);
		break;
	case MsgEnum::DSRCmsgID_spat:
		pMessageFrame->value.present = MessageFrame__value_PR_SPAT;
		tf2msgFrame = spat2msgFrame(dsrcFrameIn.spat, pMessageFrame->value.choice.SPAT);
		break;
	case MsgEnum::DSRCmsgID_bsm:
		pMessageFrame->value.present = MessageFrame__value_PR_BasicSafetyMessage;
		tf2msgFrame = bsm2msgFrame(dsrcFrameIn.bsm, pMessageFrame->value.choice.BasicSafetyMessage);
		break;
	case MsgEnum::DSRCmsgID_rtcm:
		pMessageFrame->value.present = MessageFrame__value_PR_RTCMcorrections;
		tf2msgFrame = rtcm2msgFrame(dsrcFrameIn.rtcm, pMessageFrame->value.choice.RTCMcorrections);
		break;
	case MsgEnum::DSRCmsgID_srm:
		pMessageFrame->value.present = MessageFrame__value_PR_SignalRequestMessage;
		tf2msgFrame = srm2msgFrame(dsrcFrameIn.srm, pMessageFrame->value.choice.SignalRequestMessage);
		break;
	case MsgEnum::DSRCmsgID_ssm:
		pMessageFrame->value.present = MessageFrame__value_PR_SignalStatusMessage;
		tf2msgFrame = ssm2msgFrame(dsrcFrameIn.ssm, pMessageFrame->value.choice.SignalStatusMessage);
		break;
	default:
		tf2msgFrame = false;
		break;
	}
	if (!tf2msgFrame)
	{
		ASN_STRUCT_FREE(asn_DEF_MessageFrame, pMessageFrame);
		return(0);
	}
	pMessageFrame->messageId = dsrcFrameIn.dsrcMsgId;
	asn_enc_rval_t rval = uper_encode_to_buffer(&asn_DEF_MessageFrame, 0, pMessageFrame, buf, size);
	ASN_STRUCT_FREE(asn_DEF_MessageFrame, pMessageFrame);
	return(numbits2numbytes(rval.encoded));
}

/// SAE J2735 UPER decoding function
size_t AsnJ2735Lib::decode_msgFrame(const uint8_t* buf, size_t size, Frame_element_t& dsrcFrameOut)
{
	dsrcFrameOut.reset();
	MessageFrame_t* pMessageFrame = NULL;
	asn_dec_rval_t rval = uper_decode(0, &asn_DEF_MessageFrame, (void **)&pMessageFrame, buf, size, 0, 0);
	if (rval.code != RC_OK)
	{
		std::cerr << "decode_msgFrame: failed UPER decoding" << std::endl;
		ASN_STRUCT_FREE(asn_DEF_MessageFrame, pMessageFrame);
		return(0);
	}
	/// convert decoded result to dsrcFrameOut, based on message ID
	uint16_t dsrcMsgId = static_cast<uint16_t>(pMessageFrame->messageId);
	bool tf2out;
	switch(dsrcMsgId)
	{
	case MsgEnum::DSRCmsgID_map:
		tf2out = ((pMessageFrame->value.present == MessageFrame__value_PR_MapData)
			&& msgFrame2mapData(pMessageFrame->value.choice.MapData, dsrcFrameOut.mapData));
		break;
	case MsgEnum::DSRCmsgID_spat:
		tf2out = ((pMessageFrame->value.present == MessageFrame__value_PR_SPAT)
				&& msgFrame2spat(pMessageFrame->value.choice.SPAT, dsrcFrameOut.spat));
		break;
	case MsgEnum::DSRCmsgID_bsm:
		tf2out = ((pMessageFrame->value.present == MessageFrame__value_PR_BasicSafetyMessage)
				&& msgFrame2bsm(pMessageFrame->value.choice.BasicSafetyMessage, dsrcFrameOut.bsm));
		break;
	case MsgEnum::DSRCmsgID_rtcm:
		tf2out = ((pMessageFrame->value.present == MessageFrame__value_PR_RTCMcorrections)
				&& msgFrame2rtcm(pMessageFrame->value.choice.RTCMcorrections, dsrcFrameOut.rtcm));
		break;
	case MsgEnum::DSRCmsgID_srm:
		tf2out = ((pMessageFrame->value.present == MessageFrame__value_PR_SignalRequestMessage)
				&& msgFrame2srm(pMessageFrame->value.choice.SignalRequestMessage, dsrcFrameOut.srm));
		break;
	case MsgEnum::DSRCmsgID_ssm:
		tf2out = ((pMessageFrame->value.present == MessageFrame__value_PR_SignalStatusMessage)
				&& msgFrame2ssm(pMessageFrame->value.choice.SignalStatusMessage, dsrcFrameOut.ssm));
		break;
	default:
		tf2out = false;
		break;
	}
	ASN_STRUCT_FREE(asn_DEF_MessageFrame, pMessageFrame);
	if (tf2out)
	{
		dsrcFrameOut.dsrcMsgId = dsrcMsgId;
		return(numbits2numbytes(rval.consumed));
	}
	return(0);
}
