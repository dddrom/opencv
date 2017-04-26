#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>

#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/alarm.h>

 
// Définir le GPIO g en tant qu'entrée :
// (toujours utiliser INP_GPIO(g) avant OUT_GPIO(g) ou SET_GPIO_ALT(g,f))
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))

// Définir le GPIO g en tant que sortie :
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

// Définir la fonction du GPIO g :
#define SET_GPIO_ALT(g,f) *(gpio+(((g)/10))) |= (((f)<=3?(f)+4:(f)==4?3:2)<<(((g)%10)*3))
 
// Mettre à 1 ou à 0 le GPIO correspondant au poids du bit à 1. Les 0 sont ignorés.
#define GPIO_SET *(gpio+7)
#define GPIO_CLR *(gpio+10)
 
// Accéder à la valeur du GPIO g :
// (retourne ( 1 << g ) si à 5V, 0 si à 0V )
#define GET_GPIO(g) (*(gpio+13)&(1<<g))
 
// Adresse de base pour les registres correspondant aux GPIO de la Raspberry Pi :
volatile unsigned* gpio;

 
// Définition de l'adresse de base pour l'utilisation directe des GPIO de la Raspberry Pi :
void setup_gpio_address()
{
	int mem_fd;
	void* gpio_map;

	// Accès à la mémoire physique :
	if ( ( mem_fd = open( "/dev/mem", O_RDWR | O_SYNC ) ) < 0 )
	{
		perror( "can't open /dev/mem" );
		exit( -1 );
	}

	// Création d'une zone d'adressage vers la mémoire physique correspondant
	// aux registres des GPIO :
	gpio_map = mmap(
		NULL,
		4*1024,
		PROT_READ | PROT_WRITE,
		MAP_SHARED,
		mem_fd,
		0x3F200000
	);

	
	if ( gpio_map == MAP_FAILED )
	{
		perror( "mmap" );
		exit( -1 );
	}

	gpio = (volatile unsigned*) gpio_map;
	close( mem_fd );

}


// Numéros des pins utilisés :
#define PWM_PIN_1 23
 

// Structure utilisée pour définir une sortie PWM :
typedef struct pwm_pulse {
	int pin; // numéro du pin
	RTIME width; // largeur de l'impuslion
	RT_ALARM alarm; // alarme associée
	RT_TASK task; // tâche associée
} pwm_pulse;


// Définitions globales des varaibles communes à plusieurs fonctions :
RT_TASK pwm_task; // tâche périodique
RTIME cycle_period = 20000000; // période des PWM en nanoseconde
RTIME duty_cycle = 2000000;
pwm_pulse pulse_1 = { PWM_PIN_1, 1000000 };

// Fonction utilisée pour terminer chaque impulsion :
void pwm_pulse_task( void* arg )
{
  // Interprétation de l'argument :
  pwm_pulse* pulse = (pwm_pulse*) arg;
  
  //	while ( 1 )
  //{
  //	// Attente de l'alarme correspondant à la pulsation gérée par la tâche :
  //	rt_alarm_wait( &pulse->alarm );
  
  // Mise à 0 du GPIO correspondant :
  //  GPIO_CLR = 1 << pulse->pin;
  //}
}

// Fonction générant le cycle commun des PWM :
void pwm_cycle_task( void* arg )
{
	// Création des alarmes et lancement des tâches temps réel propres à chaque PWM :
  //  rt_alarm_create( &pulse_1.alarm, "PWM ALARM 1", &pwm_pulse_task, &pulse_1 );
  
  // Définition de la tâche comme tâche périodique :
  rt_task_set_periodic( NULL, TM_NOW, cycle_period );
  
  while ( 1 )
    {
      // Attente de la fin de la période :
      rt_task_wait_period( NULL );
      
      // Mise à 1 des GPIO et lancement des alarmes :
      GPIO_SET = 1 << PWM_PIN_1;
      //      rt_alarm_start( &pulse_1.alarm, pulse_1.width, 0 );
      rt_task_sleep(duty_cycle);
      GPIO_CLR = 1 << PWM_PIN_1;
      
    }
}

// Fonction appelée lorsque le programme reçoit le signal SIGINT (ctrl+c dans un terminal) :
void sigint_handler( int sig )
{
	// Arrêt de la tâche temps réel principale :
	rt_task_delete( &pwm_task );

	// Arrêt des tâches temps réel associées à chaque PWM :
	//	rt_task_delete( &pulse_1.task );
	GPIO_CLR = 1 << PWM_PIN_1;

	printf( "\n" );
	//exit( 0 );
	
}

// Fonction initiale du programme, lancée dans l'espace utilisateur :
int main( int argc, char* argv[] )
{
	// Désactivation de la mémoire swap :
	mlockall( MCL_CURRENT | MCL_FUTURE );

	// Initialisation du pointeur vers les registres des GPIO :
	setup_gpio_address();

	// Déclaration des GPIO en tant que sorties :
	INP_GPIO( PWM_PIN_1 );
	OUT_GPIO( PWM_PIN_1 );
	


	// Association de la fonction sigint_handler au signal SIGINT :
	signal( SIGINT, sigint_handler );
	
	// Création et lancement de la tâche temps réel générant le cycle des PWM :
	rt_task_spawn( &pwm_task, "PWM CYCLE", 0, 99, T_JOINABLE, &pwm_cycle_task, NULL );

	// Récupération de nouvelles valeurs d'impulsion via l'entrée standard :
	char* line = NULL;
	size_t len = 0;
	int n;
	while( 1 )
	{
		printf( "Enter new pulse between [1000-3000](µms) : " );
		if ( getline( &line, &len, stdin ) == -1 )
			continue;
		if ( sscanf( line, "%d", &n ) > 0 )
		{
			pulse_1.width = n*1000;
			duty_cycle= n*1000;
		}
	}

	// Attente de la fin de la tâche temps réel principale :
	rt_task_join( &pwm_task );

	return 0;
}
