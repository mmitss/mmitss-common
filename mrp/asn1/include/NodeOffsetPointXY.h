//************************************************************************************************************
//
// © 2016-2019 Regents of the University of California on behalf of the University of California at Berkeley
//       with rights granted for USDOT OSADP distribution with the ECL-2.0 open source license.
//
//*************************************************************************************************************
/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../j2735_asn/J2735_201603DA.asn"
 * 	`asn1c -fcompound-names -gen-PER -gen-OER -pdu=auto`
 */

#ifndef	_NodeOffsetPointXY_H_
#define	_NodeOffsetPointXY_H_


#include <asn_application.h>

/* Including external dependencies */
#include "Node-XY-20b.h"
#include "Node-XY-22b.h"
#include "Node-XY-24b.h"
#include "Node-XY-26b.h"
#include "Node-XY-28b.h"
#include "Node-XY-32b.h"
#include "Node-LLmD-64b.h"
#include "RegionalExtension.h"
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum NodeOffsetPointXY_PR {
	NodeOffsetPointXY_PR_NOTHING,	/* No components present */
	NodeOffsetPointXY_PR_node_XY1,
	NodeOffsetPointXY_PR_node_XY2,
	NodeOffsetPointXY_PR_node_XY3,
	NodeOffsetPointXY_PR_node_XY4,
	NodeOffsetPointXY_PR_node_XY5,
	NodeOffsetPointXY_PR_node_XY6,
	NodeOffsetPointXY_PR_node_LatLon,
	NodeOffsetPointXY_PR_regional
} NodeOffsetPointXY_PR;

/* NodeOffsetPointXY */
typedef struct NodeOffsetPointXY {
	NodeOffsetPointXY_PR present;
	union NodeOffsetPointXY_u {
		Node_XY_20b_t	 node_XY1;
		Node_XY_22b_t	 node_XY2;
		Node_XY_24b_t	 node_XY3;
		Node_XY_26b_t	 node_XY4;
		Node_XY_28b_t	 node_XY5;
		Node_XY_32b_t	 node_XY6;
		Node_LLmD_64b_t	 node_LatLon;
		RegionalExtension_124P0_t	 regional;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} NodeOffsetPointXY_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_NodeOffsetPointXY;
extern asn_CHOICE_specifics_t asn_SPC_NodeOffsetPointXY_specs_1;
extern asn_TYPE_member_t asn_MBR_NodeOffsetPointXY_1[8];
extern asn_per_constraints_t asn_PER_type_NodeOffsetPointXY_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _NodeOffsetPointXY_H_ */
#include <asn_internal.h>
