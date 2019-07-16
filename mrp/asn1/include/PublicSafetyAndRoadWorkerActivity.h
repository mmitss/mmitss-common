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

#ifndef	_PublicSafetyAndRoadWorkerActivity_H_
#define	_PublicSafetyAndRoadWorkerActivity_H_


#include <asn_application.h>

/* Including external dependencies */
#include <BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PublicSafetyAndRoadWorkerActivity {
	PublicSafetyAndRoadWorkerActivity_unavailable	= 0,
	PublicSafetyAndRoadWorkerActivity_workingOnRoad	= 1,
	PublicSafetyAndRoadWorkerActivity_settingUpClosures	= 2,
	PublicSafetyAndRoadWorkerActivity_respondingToEvents	= 3,
	PublicSafetyAndRoadWorkerActivity_directingTraffic	= 4,
	PublicSafetyAndRoadWorkerActivity_otherActivities	= 5
} e_PublicSafetyAndRoadWorkerActivity;

/* PublicSafetyAndRoadWorkerActivity */
typedef BIT_STRING_t	 PublicSafetyAndRoadWorkerActivity_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_PublicSafetyAndRoadWorkerActivity_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_PublicSafetyAndRoadWorkerActivity;
asn_struct_free_f PublicSafetyAndRoadWorkerActivity_free;
asn_struct_print_f PublicSafetyAndRoadWorkerActivity_print;
asn_constr_check_f PublicSafetyAndRoadWorkerActivity_constraint;
ber_type_decoder_f PublicSafetyAndRoadWorkerActivity_decode_ber;
der_type_encoder_f PublicSafetyAndRoadWorkerActivity_encode_der;
xer_type_decoder_f PublicSafetyAndRoadWorkerActivity_decode_xer;
xer_type_encoder_f PublicSafetyAndRoadWorkerActivity_encode_xer;
oer_type_decoder_f PublicSafetyAndRoadWorkerActivity_decode_oer;
oer_type_encoder_f PublicSafetyAndRoadWorkerActivity_encode_oer;
per_type_decoder_f PublicSafetyAndRoadWorkerActivity_decode_uper;
per_type_encoder_f PublicSafetyAndRoadWorkerActivity_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _PublicSafetyAndRoadWorkerActivity_H_ */
#include <asn_internal.h>
