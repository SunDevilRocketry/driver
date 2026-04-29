# Calibration coefficients
par_p1 = -0.00868797302
par_p2 = -1.95223838e-05
par_p3 = 1.39698386e-09 
par_p4 = 7.27595761e-12 
par_p5 = 154288 
par_p6 = 365.09375 
par_p7 = 0.01171875 
par_p8 = -0.000183105469 
par_p9 = 1.40367717e-11 
par_p10 = 2.13162821e-14 
par_p11 = -2.98155597e-19 

# Coefficient Accuracy
P_coeff_num = input( "Enter Pressure Compensation Accuracy: ")
T_coeff_num = input( "Enter Temperature Compensation Accuracy: " )
P_coeff_num = int( P_coeff_num )
T_coeff_num = int( T_coeff_num )

# Raw readout
Praw = 8_388_608

# Temperature Measurement
temp = 29.0905704

# Put coefficients in arrays
C0 = [ par_p5, par_p6, par_p7, par_p8 ]
C1 = [ par_p1, par_p2, par_p3, par_p4 ]
C2 = [ par_p9, par_p10 ]
C3 = [ par_p11 ]
C  = [ C0, C1, C2, C3 ]

# Calculate Pressure coefficients
T_coeff = []
for Coeff in C:
	val = 0
	for i in range( T_coeff_num + 1 ):
		if ( not ( i >= len(Coeff) ) ):
			val += Coeff[i]*temp**(i)
	T_coeff.append( val )

# Calculate Pressure
pressure = 0
for i in range( P_coeff_num + 1 ):
	pressure += T_coeff[i]*(Praw**(i))

# Calculate uncompensated Pressure value
byte_msb = ( Praw & ( 0xFF << 16 ) ) >> 16
byte_lsb = ( Praw & ( 0xFF << 8  ) ) >> 8
byte_xlsb = ( Praw & ( 0xFF ) )
recomb = ( (byte_msb << 8 + 3 ) | ( byte_lsb << 3 ) | byte_xlsb )
print( recomb )

# Convert to Pa 
baro_max_press     = 1250.0*100.0                     # Pa
baro_min_press     = 300.0*100.0                      # Pa
baro_press_range   = baro_max_press - baro_min_press  # Pa
baro_press_setting = (baro_press_range/(2**(19) - 1)) # Pa/LSB 
baro_press         = float(recomb)*baro_press_setting + baro_min_press # Pa
baro_press        /= 1000 # kPa

# Display Results
print()
print( "Raw Pressure: " + str( baro_press ) + " kPa" )
print( "Pressure    : " + str( pressure/1000 ) + " kPa" )
