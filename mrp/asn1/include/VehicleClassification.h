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

#ifndef	_VehicleClassification_H_
#define	_VehicleClassification_H_


#include <asn_application.h>

/* Including external dependencies */
#include "BasicVehicleClass.h"
#include "BasicVehicleRole.h"
#include "Iso3833VehicleType.h"
#include "VehicleType.h"
#include "VehicleGroupAffected.h"
#include "IncidentResponseEquipment.h"
#include "ResponderGroupAffected.h"
#include "FuelType.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct RegionalExtension;

/* VehicleClassification */
typedef struct VehicleClassification {
	BasicVehicleClass_t	*keyType	/* OPTIONAL */;
	BasicVehicleRole_t	*role	/* OPTIONAL */;
	Iso3833VehicleType_t	*iso3883	/* OPTIONAL */;
	VehicleType_t	*hpmsType	/* OPTIONAL */;
	VehicleGroupAffected_t	*vehicleType	/* OPTIONAL */;
	IncidentResponseEquipment_t	*responseEquip	/* OPTIONAL */;
	ResponderGroupAffected_t	*responderType	/* OPTIONAL */;
	FuelType_t	*fuelType	/* OPTIONAL */;
	struct VehicleClassification__regional {
		A_SEQUENCE_OF(struct RegionalExtension) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *regional;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} VehicleClassification_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_VehicleClassification;
extern asn_SEQUENCE_specifics_t asn_SPC_VehicleClassification_specs_1;
extern asn_TYPE_member_t asn_MBR_VehicleClassification_1[9];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "RegionalExtension.h"

#endif	/* _VehicleClassification_H_ */
#include <asn_internal.h>
