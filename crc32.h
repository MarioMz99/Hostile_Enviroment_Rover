#include "driver/uart.h"

#define UARTS_BAUD_RATE         (115200)
#define TASK_STACK_SIZE         (1048)
#define READ_BUF_SIZE           (1024)

//Estructura para el paquete
typedef struct UARTPackage{
    uint8_t header;
    uint8_t command;
    uint8_t length;
    uint8_t end;
    uint8_t *data;
    uint32_t crc32;
} UARTPackage;

void agregar_en_arreglo(char *arr, int size, char value);

uint32_t crc32b(char *message); ///Calcular el crc32

//UART Package functions
UARTPackage Crear_paquete(uint8_t header, uint8_t command, uint8_t length, uint8_t *data, uint8_t end);
UARTPackage decodificar_estructura(char *str);
void codificar_estructura(char *str, UARTPackage pkg);
void imprime_estructura(UARTPackage pkg);