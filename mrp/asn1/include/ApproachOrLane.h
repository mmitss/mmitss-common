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

#ifndef	_ApproachOrLane_H_
#define	_ApproachOrLane_H_


#include <asn_application.h>

/* Including external dependencies */
#include "ApproachID.h"
#include "LaneID.h"
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ApproachOrLane_PR {
	ApproachOrLane_PR_NOTHING,	/* No components present */
	ApproachOrLane_PR_approach,
	ApproachOrLane_PR_lane
} ApproachOrLane_PR;

/* ApproachOrLane */
typedef struct ApproachOrLane {
	ApproachOrLane_PR present;
	union ApproachOrLane_u {
		ApproachID_t	 approach;
		LaneID_t	 lane;
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ApproachOrLane_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ApproachOrLane;
extern asn_CHOICE_specifics_t asn_SPC_ApproachOrLane_specs_1;
extern asn_TYPE_member_t asn_MBR_ApproachOrLane_1[2];
extern asn_per_constraints_t asn_PER_type_ApproachOrLane_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _ApproachOrLane_H_ */
#include <asn_internal.h>
