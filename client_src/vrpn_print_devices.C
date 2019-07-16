#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <vrpn_Connection.h>

// Defines
#define VRPN_SERVER_IP          "192.168.10.1"
#define VRPN_TARGET_NAME        "wiimote"
#define VRPN_TRACKER_TYPE       "vrpn_Tracker Pos_Quat"
#define VRPN_MSG_LEN            8
#define VRPN_TRANS_LEN          3
#define VRPN_ROT_LEN            4

using namespace std;

int my_handler( void *userdata, vrpn_HANDLERPARAM p ) {
  int           i;
	const char    *param = (p.buffer);
	vrpn_float64  pos[VRPN_TRANS_LEN], 
	              quat[VRPN_ROT_LEN];
  vrpn_int32    sensor_number;

	// Timestamp (us) management
	static char               first_flag = 1;
	static unsigned long long first_time;

	unsigned long long        usec =  p.msg_time.tv_sec * 1e6 + 
	                                  p.msg_time.tv_usec;

	if ( first_flag )  {  
	       first_time = usec;
	       first_flag = 0;
	  }
	  
	unsigned long long        timestamp = usec - first_time; // timestamp in us

	// Check for errors
	if ( p.payload_len != ( VRPN_MSG_LEN * sizeof( vrpn_float64 ) ) ) {
		fprintf(stderr,"vrpn_Tracker: change message payload error\n");
		fprintf(stderr,"             (got %d, expected %lud)\n",
			p.payload_len, 
			static_cast<unsigned long>( VRPN_MSG_LEN * sizeof(vrpn_float64) ) );
		return -1;
	}

  // Extract data
	sensor_number = ntohl( *((vrpn_int32*)(param)) );
	for ( i = 0; i < VRPN_TRANS_LEN; i++ )
		pos[i] = ntohd( *( (vrpn_float64*)( param+((1+i)*sizeof(vrpn_float64)) ) ) );

	for ( i = 0; i < VRPN_ROT_LEN; i++ ) {
		quat[i] = ntohd( *( (vrpn_float64*)( param+((4+i)*sizeof(vrpn_float64)) ) ) );
	}

  // Display pose
	printf( "Tracker %i : time ms (%lli) pos (%lf,%lf,%lf) quat (%lf,%lf,%lf,%lf)\n",
	  sensor_number, timestamp, 
	  pos[0], pos[1], pos[2],
	  quat[0], quat[1], quat[2], quat[3] );

	return 0;
}

// Main
int main ( int argc, char * argv [] ) {

vrpn_Connection *connection =  vrpn_get_connection_by_name( VRPN_SERVER_IP );

long my_id = connection->register_sender( VRPN_TARGET_NAME );
long my_type = connection->register_message_type( VRPN_TRACKER_TYPE );

connection->register_handler( my_type, my_handler, (void*)"None", my_id );

while ( 1 )
  connection->mainloop( );

return 0;
}

