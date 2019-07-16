#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <strings.h>
#include <arpa/inet.h> // for ntohl [,d]

#include <vrpn_Connection.h>  // For preload and accumulate settings
				//Only include from vrpn lib
#include <unistd.h> //for sleep



using namespace std;


int my_handler(void *userdata, vrpn_HANDLERPARAM p) {
	int i;
	const char *param = (p.buffer);
	vrpn_float64 pos[3], quat[4]; //double
        vrpn_int32  sensor_number;

	//timestamp (us) management
	static char first =1;
	static long long first_time;

	long long sec=p.msg_time.tv_sec*1e6;
	long long usec=p.msg_time.tv_usec;

	if (first)
	  {  
	       first_time=(sec+usec)/1000;
	       first=0;
	  }
	long long timestamp=(sec+usec)/1000-first_time; // timestamp in ms

	//en timestamp mangement --------------------------------

	
	if (p.payload_len != (8*sizeof(vrpn_float64)) ) {
		fprintf(stderr,"vrpn_Tracker: change message payload error\n");
		fprintf(stderr,"             (got %d, expected %lud)\n",
			p.payload_len, static_cast<unsigned long>(8*sizeof(vrpn_float64)) );
		return -1;
	}

	sensor_number=ntohl(*((vrpn_int32*)(param)));
	for (i = 0; i < 3; i++)
		pos[i]=ntohd( *( (vrpn_float64*) (param+((1+i)*sizeof(vrpn_float64)) )));

	for (i = 0; i < 4; i++) {
		quat[i]=ntohd( *( (vrpn_float64*) (param+((4+i)*sizeof(vrpn_float64)) )));
	}


		printf("Tracker %i : time ms (%lli) pos (%lf,%lf,%lf) quat (%lf,%lf,%lf,%lf)\n",sensor_number, timestamp, pos[0],pos[1],pos[2],quat[0], quat[1], quat[2], quat[3]);

		return 0;
	}



int main (int argc, char * argv [])
{

vrpn_Connection *connection =  vrpn_get_connection_by_name("192.168.10.1");

long my_id = connection->register_sender("wiimote"); //indispensable Tracker0 for test
long my_type = connection->register_message_type("vrpn_Tracker Pos_Quat");

connection->register_handler(my_type, my_handler, (void*)"None", my_id);

while (1)
connection->mainloop();

return 0;
}

