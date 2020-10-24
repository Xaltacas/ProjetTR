#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <asm/io.h>

#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include "driver_util.h"

MODULE_LICENSE( "GPL" );
MODULE_DESCRIPTION( "Pilote pour récupération Interruptions sur Raspberry Pi" );



#define  CLASS_NAME  "char_motor"
int major=MAJOR_NUM;
static struct class*  charClass  = NULL;
static struct device* charDevice;
const char* device_name ="motor";


// Numéros des pins utilisés :
#define IRQ_PIN_0 5
#define IRQ_PIN_1 6


// Définition des macros pour l'utilisation directe des GPIO de la Raspberry Pi :
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
#define OUT_GPIO(g)
#define GET_GPIO(g) (*(gpio+13)&(1<<g))

#define SIGN(g) ((GET_GPIO(g)>>(g-1))-1) // equivalent à GET_GPIO(g)? 1:-1

// Adresse de base pour les registres correspondant aux GPIO de la Raspberry Pi :
static volatile unsigned* gpio;


// Les variables globales et les fonctions sont déclarées statiques afin qu'elles ne soient visibles
// que dans le code de ce fichier. On évite ainsi toute ambiguïté avec autres variables du noyau
// qui partage le même segment de mémoire.

// Valeur correspondant à la position du codeur :
static volatile long encoder_count = 0;

// Déclaration des numéros de pins comme paramètres du module :
static const int irq_pin[2] = {IRQ_PIN_0, IRQ_PIN_1};

//module_param( irq_pin_1, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP );


// Structure utilisée pour traiter les interruptions :
typedef struct encoder_data {
    char* label;
    int irq_pin;
    long* count;
    unsigned int irq;
} encoder_data;

// Déclaration des structures de données utilisées pour traiter les interruptions :
static encoder_data encoder[2];




//-------------------------------------
// Fonctions gérant les interruptions :
//-------------------------------------

// Fonction appelée à chaque interruption :
static irqreturn_t encoder_irq_handler_0( int irq, void* dev_id )
{
    // Interprétation du pointeur vers les données de l'interruption :
    //encoder_data* data = (encoder_data*) dev_id;

    // TODO : Incrémentation du codeur.
    // GET_GPIO( numéro_de_pin ) retourne ( 1 << numéro_du_pin ) si le pin est à 5V, 0 s'il est à 0V.
    int inc = SIGN(irq_pin[0]) // rising ou falling
        * SIGN(irq_pin[1]); // l'autre channel
    //(data->count) -= inc;
    encoder_count += inc;
    return IRQ_HANDLED;
}


static irqreturn_t encoder_irq_handler_1( int irq, void* dev_id ){
    encoder_count -= SIGN(irq_pin[0]) * SIGN(irq_pin[1]);
    return IRQ_HANDLED;
}

typedef irqreturn_t (*irq_handler)(int, void*);
irq_handler handlers[2]={&encoder_irq_handler_0, &encoder_irq_handler_1};


// Fonction pour allouer les pins : c==0 ou c==1
static void setup_irq_channel( int c)//encoder_data* data )
{
    encoder[c].irq_pin = irq_pin[c];
    encoder[c].count = (long*)&encoder_count;
    // Initialisations propre à la Raspberry Pi :
    INP_GPIO( encoder[c].irq_pin );
    SET_GPIO_ALT( encoder[c].irq_pin, 0 );

    // Allocation de l'interruption :
    gpio_request_one( encoder[c].irq_pin, GPIOF_IN, encoder[c].label ); // Réservation système en input

    encoder[c].irq = gpio_to_irq( encoder[c].irq_pin ); // Recherche du numéro d'interruptionà partir du numéro de pin

    if(request_any_context_irq(
        encoder[c].irq,
        *handlers[c],
        IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
        THIS_MODULE->name,
        encoder+c) < 0)
            printk( KERN_ALERT " interruption allocation failed");
        //printk( KERN_INFO "%s: interruption \"%s\" allocated on line %u\n", THIS_MODULE->name, data->label, data->irq );
}

// Fonction pour libérer les pins :
static void free_encoder_pins( encoder_data* data )
{
    free_irq( data->irq, data );
    gpio_free( data->irq_pin );

}


static ssize_t char_read(struct file *file, char *buf, size_t count, loff_t *ppos){
    //put_user(encoder_count,(long *)buf);
  return count;
}

static ssize_t char_write(struct file *file, const char *buf, size_t count, loff_t *ppos){
    return 0;
}

static int char_open(struct inode *inode, struct file *file){
    //printk(KERN_INFO "oPeN");
  return 0;
}

static int char_release(struct inode *inode, struct file *file){
    //printk(KERN_INFO "REaLeasE");
  return 0;
}

static long char_ioctl(struct file *file, unsigned int cmd, unsigned long arg){
    
    if(cmd!=IOCTL_GET_COUNT) return 1;
    put_user(encoder_count, (long *)arg);
    //copy_to_user((int *)arg, &encoder_count, 4); // 4 bytes
    return 0;
}

static struct file_operations char_fops = {
    .owner =    THIS_MODULE,
    .read =        char_read,
    .write =    char_write,
    .unlocked_ioctl =    char_ioctl,
    .open =        char_open,
    .release =    char_release,
};


//-----------------------------
// Fonctions gérant le module :
//-----------------------------

// Fonction appelée au chargement du module :
int init_module( void )
{
    void* gpio_map;
    int ret;

    // Translation des adresses pour l'utilisation directe des GPIO de la Raspberry Pi :
    gpio_map = ioremap( GPIO_BASE, GPIO_LEN);//SZ_16K );
    if ( gpio_map == NULL )
    {
        printk( KERN_ALERT "%s: ioremap failed !\n", THIS_MODULE->name );
        return -EBUSY;
    }
    gpio = (volatile unsigned*) gpio_map;
    
    encoder[0].label = "channel A";
    encoder[1].label = "channel B";

    // Allocation des pins :
    //setup_irq_pin( &data_1 );
    setup_irq_channel( 0 );
    setup_irq_channel( 1 );
    
    
    
    ret = register_chrdev(major, device_name, &char_fops);
    if (ret<0){
        printk(KERN_INFO "major failed");
      return ret;
    }
    else if (ret>0) major = ret;
    //printk(KERN_INFO "Major %d\n", major);

    /****** Automatically creating virtual files in /dev ******/
    // Register the device class
    charClass = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(charClass)){                // Check for error and clean up if there is
      unregister_chrdev(major, device_name);
      printk(KERN_ALERT "Failed to register device class\n");
      return PTR_ERR(charClass);          // Correct way to return an error on a pointer
    }
    
    // Register the device driver
    charDevice = device_create(charClass, NULL, MKDEV(major, 0), NULL, device_name);
    if (IS_ERR(charDevice)){               // Clean up if there is an error
      class_destroy(charClass);           // Repeated code but the alternative is goto statements
      unregister_chrdev(major, device_name);
        printk(KERN_ALERT "Failed to create device\n");
      return PTR_ERR(charDevice);
    }
      

    return 0;
}

// Fonction appelée au retrait du module :
void cleanup_module( void )
{

    // Libération des pins :
    free_encoder_pins( encoder );
    free_encoder_pins( encoder+1 );
    iounmap((void*)gpio);
    
    device_destroy(charClass, MKDEV(major, 0));
    class_unregister(charClass);
    class_destroy(charClass);
    unregister_chrdev(major, "mychar");

    printk( KERN_INFO "%s: module removed\n", THIS_MODULE->name );
}
