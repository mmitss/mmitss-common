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

#ifndef	_RoadSegmentReferenceID_H_
#define	_RoadSegmentReferenceID_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RoadRegulatorID.h"
#include "RoadSegmentID.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RoadSegmentReferenceID */
typedef struct RoadSegmentReferenceID {
	RoadRegulatorID_t	*region	/* OPTIONAL */;
	RoadSegmentID_t	 id;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RoadSegmentReferenceID_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RoadSegmentReferenceID;
extern asn_SEQUENCE_specifics_t asn_SPC_RoadSegmentReferenceID_specs_1;
extern asn_TYPE_member_t asn_MBR_RoadSegmentReferenceID_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _RoadSegmentReferenceID_H_ */
#include <asn_internal.h>
