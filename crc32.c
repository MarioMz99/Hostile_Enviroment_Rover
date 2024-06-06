#include "esp_log.h"
#include "stdlib.h"
#include "string.h"
#include "crc32.h"

void agregar_en_arreglo(char *arr, int size, char value){
    int new_size = size + 1;
    
    arr = (char*) realloc(arr, new_size * (int)sizeof(char));
    arr[new_size - 1] = value;
}

//PARA EL CRC32
uint32_t crc32b(char *message) {
   int i, j;
   unsigned int byte, crc, mask;

   i = 0;
   crc = 0xFFFFFFFF;
   while (message[i] != 0xB2) {
      byte = message[i];            // Get next byte.
      crc = crc ^ byte;
      for (j = 0; j <8 ; j++) {    // Do eight times.
         mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320 & mask);
      }
      i = i + 1;
   }
   return ~crc;
}

UARTPackage Crear_paquete(uint8_t header, uint8_t command, uint8_t length, uint8_t *data, uint8_t end){
    UARTPackage pkg;
    
    pkg = (UARTPackage) {
        .header = header, 
        .command = command, 
        .length = length,
        .end = end,
        .crc32 = 0
    };
    if(length > 0){
        pkg.data = (uint8_t*) malloc(sizeof(uint8_t) * length);
        memcpy(pkg.data, data, length+1);
    }else{
        pkg.data = NULL;
    }

    return pkg;
}

///Descompone la estructura recibida
UARTPackage decodificar_estructura(char *str){
    UARTPackage pkg;
    int i = 0;
    uint8_t pos = 0, auxVar = 0;
    uint32_t crc32 = 0;

    pkg.header  = ((uint8_t) str[pos++]); //pos 1
    pkg.command = ((uint8_t) str[pos++]); //pos 1
    pkg.length = (((uint8_t) str[pos++])); //pos 3

    if(pkg.length > 0){ //si es que tiene valor
        pkg.data = (uint8_t*) malloc(sizeof(uint8_t) * pkg.length + 1); //tomamos la posicion donde empieza data 
        for(i=0; i < pkg.length; i++){
            pkg.data[i] = str[pos++];
        }
        pkg.data[i] = '\0';
    }else{
        pkg.data = NULL;
    }
    pkg.end = ((uint8_t) str[pos]); //guardamos lo ultimo
    
    for(int i = 0; i < 3; i++){ 
        auxVar = ((uint8_t) str[pos++]);
        crc32 |= auxVar;
        crc32 <<= 8;
    }
    crc32 |= ((uint8_t) str[pos]);
    
    pkg.crc32 = ((crc32 >> 24) & 0xFF) | ((crc32 << 8) & 0xFF0000) | ((crc32 >> 8) & 0xFF00) | ((crc32 << 24) & 0xFF000000);
    return pkg;
}

void codificar_estructura(char *str, UARTPackage pkg){
    int size = 0;
    uint32_t crc32 = 0;

    agregar_en_arreglo(str, size++, (char) pkg.header);
    agregar_en_arreglo(str, size++, (char) pkg.command);
    agregar_en_arreglo(str, size++, (char) (pkg.length == 0x0 ? -0x1 : pkg.length));

    for(int i = 0; i < pkg.length; i++){
        agregar_en_arreglo(str, size++, (char)pkg.data[i]);
    }

    agregar_en_arreglo(str, size++, (char) pkg.end);
    pkg.crc32 = crc32b(str);

    crc32 = pkg.crc32;

    for(int i = 0 ; i < 4; i++){
        agregar_en_arreglo(str, size++, (char) crc32);
        crc32 >>= 8;
    }
}