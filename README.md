#Firmware für HMW_LC_Sw2_DR RS485 Modul: 
* Homematic Wired Hombrew Hardware Arduino nano als Homematic-Device 2 Taster Eingänge und 2 Relayausgänge nach einer Vorlage von Thorsten Pferdekaemper  (thorsten@pferdekaemper.com) und Dirk Hoffmann (hoffmann@vmd-jena.de)  
* Das Modul kann ohne zusätzliche CCU-addon an einer CCU betrieben werden 
*mit Erweiterung um eine Identfy LED
===================================

#Hardwarebeschreibung: 
* Pinsettings for Arduino nano 
* D0: RXD, normaler Serieller Port fuer Debug-Zwecke und Firmware 
* D1: TXD, normaler Serieller Port fuer Debug-Zwecke und Firmware 
* D2: Direction (DE/-RE) Driver/Receiver Enable vom RS485-Treiber 
* D5: RXD, RO des RS485-Treiber 
* D6: TXD, DI des RS485-Treiber 
* D8: Button * D12: Identfy LED 
* D13: Button State LED 
* A0: Eingang 1 
* A1: Eingang 2 
* A2: Ausgang 1 
* A2: Ausgang 2
