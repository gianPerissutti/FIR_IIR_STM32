
import serial
import os
import time
fin_exec = 0

puerto_com = input("Ingrese el número del puerto COM (por ejemplo, 6 para COM6): ")
puerto_str = f'COM{puerto_com}'
# Crea un objeto Serial con la configuración adecuada para el puerto COM elegido
try:    
    puerto = serial.Serial(puerto_str, baudrate=57600, timeout=1)
except serial.SerialException as e:
    print(f"Error al abrir el puerto {puerto_str}: {e}")
    fin_exec = 1
finally:

    while(fin_exec <= 0):
        print("Menu de comandos:")
        print("-Ingrese 'F' para cambiar a filtro FIR" )
        print("-Ingrese 'I' para cambiar a filtro IIR" )
        print("-Ingrese 'LP' para cambiar a filtro pasabajos" )
        print("-Ingrese 'BP' para cambiar a filtro pasabanda" )
        print("-Ingrese 'HP' para cambiar a filtro pasaaltos" )
        print("-Ingrese 'M' para guardar 250 valores en archivo de texto")
        print("-Ingrese 'S' para salir")

        comando = input("Ingrese un comando:")

        if(comando == 'M'):

            puerto.write(bytearray('data_req','ascii'))
            print("Directorio de trabajo actual:", os.getcwd())

            with open('data.txt', 'a') as archivo:
                 ultimo_dato_tiempo = time.time()
                 read_data=1
                 while (read_data==1):
                    data = puerto.readline()
                    datos_decodificados = data.decode('utf-8').strip()
                    print(datos_decodificados)
                    archivo.write(datos_decodificados + '\n')

                    if puerto.in_waiting == 0:
                        tiempo_actual = time.time()
                        if tiempo_actual - ultimo_dato_tiempo >= 1:
                            print("No se reciben más datos. Cerrando el archivo...")
                            archivo.close()
                            read_data=0
                        else:
                            ultimo_dato_tiempo = time.time() 

        if(comando == 'I'):
           puerto.write(bytearray('IIR_req','ascii'))
           data = puerto.readline()
           datos_decodificados = data.decode('utf-8').strip()
           print(datos_decodificados)

        if(comando == 'F'):
            puerto.write(bytearray('FIR_req','ascii'))
            data = puerto.readline()
            datos_decodificados = data.decode('utf-8').strip()
            print(datos_decodificados)

        if(comando == 'LP'):
            puerto.write(bytearray('LPf_req','ascii'))
            data = puerto.readline()
            datos_decodificados = data.decode('utf-8').strip()
            print(datos_decodificados)
        

        if(comando == 'BP'):
            puerto.write(bytearray('BPf_req','ascii'))
            data = puerto.readline()
            datos_decodificados = data.decode('utf-8').strip()
            print(datos_decodificados)
         

        if(comando == 'HP'):
            puerto.write(bytearray('HPf_req','ascii'))
            data = puerto.readline()
            datos_decodificados = data.decode('utf-8').strip()
            print(datos_decodificados)
        
        
        if(comando == 'S'):           
            fin_exec = 1
            puerto.close()
        