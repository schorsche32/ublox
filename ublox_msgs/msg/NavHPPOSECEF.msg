# NAV-HPPOSECEF (0x01 0x13)
# Position Solution in ECEF
#
# See important comments concerning validity of position given in section
# Navigation Output Filters.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 19

uint8 version           # Message version (0x00 for this version).
uint8 reserved0         # Reserved
uint32 i_tow            # GPS Millisecond Time of Week [ms]
int32 ecef_x            # ECEF X coordinate [cm]
int32 ecef_y            # ECEF Y coordinate [cm]
int32 ecef_z            # ECEF Z coordinate [cm]
int8 ecef_x_hp          # High precision component of ECEF X coordinate. Must
                        # be in the range of -99..+99. Precise coordinate in cm =
                        # ecef_x + (ecef_x_hp * 1e-2). [0.1 mm]
int8 ecef_y_hp          # High precision component of ECEF X coordinate. Must
                        # be in the range of -99..+99. Precise coordinate in cm =
                        # ecef_y + (ecef_y_hp * 1e-2). [0.1 mm]
int8 ecef_z_hp          # High precision component of ECEF X coordinate. Must
                        # be in the range of -99..+99. Precise coordinate in cm =
                        # ecef_z + (ecef_z_hp * 1e-2). [0.1 mm]
int8 flags              # Flags
                        # bit 0: 1 = Invalid ecef_x, ecef_y, ecef_z, ecef_x_hp, ecef_y_hp and ecef_z_hp
                        # bit 1: Reserved
                        # bit 2: Reserved
                        # bit 3: Reserved
                        # bit 4: Reserved
                        # bit 5: Reserved
                        # bit 6: Reserved
                        # bit 7: Reserved
uint32 p_acc            # Position Accuracy Estimate [0.1 mm]