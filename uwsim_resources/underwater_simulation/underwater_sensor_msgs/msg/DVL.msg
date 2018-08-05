Header header
string header_dvl
string date 
float64 salinity 
float64 temperature   	# TODO::ARNAU ha de ser generic ?
float64 depth  		# TODO::ARNAU ha de ser generic
float64 sound_speed 
int32 test
#WATER-MASS, INSTRUMENT-REFERENCED VELOCITY DATA
float64 wi_x_axis 
float64 wi_y_axis 
float64 wi_z_axis 
float64 wi_error 
string wi_status 
#BOTTOM-TRACK, INSTRUMENT-REFERENCED VELOCITY DATA
float64 bi_x_axis 
float64 bi_y_axis 
float64 bi_z_axis 
float64 bi_error 
string bi_status 
#WATER-MASS, SHIP-REFERENCED VELOCITY DATA
float64 ws_transverse 
float64 ws_longitudinal 
float64 ws_normal 
string ws_status 
#BOTTOM-TRACK, SHIP-REFERENCED VELOCITY DATA
float64 bs_transverse 
float64 bs_longitudinal 
float64 bs_normal 
string bs_status 
#WATER-MASS, EARTH-REFERENCED VELOCITY DATA
float64 we_east 
float64 we_north 
float64 we_upwards 
string  we_status 
#BOTTOM-TRACK, EARTH-REFERENCED VELOCITY DATA
float64 be_east 
float64 be_north 
float64 be_upwards 
string  be_status 
#  WATER-MASS, EARTH-REFERENCED DISTANCE DATA
float64 wd_east 
float64 wd_north 
float64 wd_upwards 
float64 wd_range 
float64 wd_time 
#BOTTOM-TRACK, EARTH-REFERENCED DISTANCE DATA
float64 bd_east 
float64 bd_north 
float64 bd_upwards 
float64 bd_range 
float64 bd_time
#RAW DATA 
string raw_data
