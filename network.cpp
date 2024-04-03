/**
 * @class NetworkManager
 * @file network.cpp
 * @author Improvement & Innovation Team (br-idpbg-engtest-support@mail.foxconn.com)
 * @version 0.1
 * @date 2024-02-29
 *
 * @copyright Copyright (c) 2024
 */

#include <WiFi.h>         // Include the WiFi library
#include <Arduino.h>      // Include the Arduino library
#include <PubSubClient.h> // Include the PubSubClient library
#include <ESPmDNS.h>      // Include the ESPmDNS library
#include <WiFiUdp.h>      // Include the WiFiUdp library
#include <ArduinoOTA.h>   // Include the ArduinoOTA library

/**
 * @brief Manages the network connection and MQTT communication.
 *
 * The NetworkManager class provides methods to create a network bridge, connect to the network and MQTT server,
 * check the connection status, publish messages, and retrieve the connection status.
 *
 * The class requires the client name, network SSID and password, MQTT server address and port, MQTT username and password
 * to be provided during initialization.
 *
 * The class uses the PubSubClient library to communicate with the MQTT server.
 * The class also uses the WiFi library to connect to the WiFi network.
 * The class uses the Arduino.h library for basic Arduino functions.
 * The class uses the String and std::string libraries for string manipulation.
 *
 * @param pin The pin to be used as an indicator of the connection status.
 * @param clientID The name of the client.
 * @param clientName The name of the client.
 * @param online The name of the client.
 * @param prevTime The previous time the device was online.
 * @param mac The MAC address of the device.
 * @param netSSID The SSID of the network to connect to.
 * @param netPass The password of the network.
 * @param mqttServer The MQTT server address.
 * @param mqttPort The MQTT server port.
 * @param mqttUser The MQTT username.
 * @param mqttPass The MQTT password.
 * @param espClient The WiFi client object.
 * @param client The PubSubClient object for MQTT communication.
 *
 */
class NetworkManager
{
private:
    // Hardware properties
    int pin;    // The pin to be used as an indicator of the connection status.
    String mac; // The MAC address of the device.
    // Client properties
    const char *clientID;            // The id of the client.
    char clientName[20] = "Device_"; // The name of the client.
    char onlineId[20] = "Online_";   // The name of the client.
    const char *password = "pass";   // The password for OTA.
    char *msgPayload;                // The payload to be sent.
    // WiFi properties
    const char *netSSID;  // The SSID of the network to connect to.
    const char *netPass;  // The password of the network.
    WiFiClient espClient; // The WiFi client object.
    // MQTT properties
    const char *mqttServer;     // The MQTT server address.
    int mqttPort;               // The MQTT server port.
    const char *mqttUser;       // The MQTT username.
    const char *mqttPass;       // The MQTT password.
    const char *topic = "none"; // The MQTT topic to subscribe to.
    // Cycle properties
    bool verbose;                          // The verbose mode flag.
    bool firstCycle = true;                // The first cycle flag.
    bool deployCycle = true;               // The deploy cycle flag.
    unsigned long currentMillis;           // The current time in milliseconds.
    unsigned long prevTime = 0;            // The previous time the device was online.
    unsigned long prevCycle = 0;           // The previous cycle time.
    unsigned long connectionTimer = 60000; // 1 minute
    unsigned long onlineCycle = 1000;      // 1 seconds
    unsigned long cycleTime = 5000;        // 5 seconds

    /**
     * @brief Restarts the device.
     *
     * This function restarts the device by calling the ESP.restart() function.
     */
    void restartDevice()
    {
        Serial.println(" Reiniciando o dispositivo...");
        ESP.restart();
    }

    /**
     * @brief Connects to the specified WiFi network.
     *
     * This function connects to the specified WiFi network using the SSID and password provided during initialization.
     * If the connection fails, the device is restarted.
     */
    void connectWiFi()
    {
        Serial.print("Conectando " + String(clientName) + " a rede WiFi: " + String(netSSID) + "..");
        WiFi.mode(WIFI_STA);
        WiFi.begin(netSSID, netPass);
        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED)
        {
            Serial.print(".");
            delay(1000);
            if (millis() - startTime > connectionTimer)
            {
                Serial.print("Falha ao conectar a rede WiFi!");
                restartDevice();
            }
        }
        Serial.println("CONECTADO!");
    }

    /**
     * @brief Connects to the specified MQTT server.
     *
     * This function connects to the specified MQTT server using the server address, port, username, and password provided during initialization.
     * If the connection fails, the device is restarted.
     */
    bool connectToMqttServer()
    {
        bool isConnected = false;
        if (mqttUser == "none" && mqttPass == "none")
        {
            isConnected = client.connect(clientName);
        }
        else
        {
            isConnected = client.connect(clientName, mqttUser, mqttPass);
        }

        if (isConnected && topic != "none")
        {
            client.subscribe(topic);
        }
        return isConnected;
    }
    /**
     * @brief Connects to the specified MQTT server.
     *
     * This function connects to the specified MQTT server using the server address, port, username, and password provided during initialization.
     * It also turns on the connection indicator LED. If the connection fails, the device is restarted.
     */
    void connectServer()
    {
        Serial.print("Conectando " + String(clientName) + " ao servidor MQTT: " + String(mqttServer) + ":" + String(mqttPort) + "..");
        unsigned long startTime = millis();
        while (!client.connected())
        {
            Serial.print(".");
            bool isConnected = connectToMqttServer();
            delay(5000);
            if (!isConnected)
            {
                if (millis() - startTime > connectionTimer)
                {
                    Serial.print("Falha ao conectar ao servidor MQTT!");
                    restartDevice();
                }
            }
        }
        digitalWrite(pin, HIGH);
        Serial.println("CONECTADO!");
    }

    /**
     * @brief Starts the OTA (Over-The-Air) update process.
     *
     * This function starts the OTA update process, allowing the device to be updated over the air.
     * It sets the hostname and password for the OTA update.
     */
    void startOTA()
    {
        connectWiFi();
        ArduinoOTA.setHostname(clientName);
        ArduinoOTA.setPassword(password);
        ArduinoOTA.begin();
        ArduinoOTA.onStart([]()
                           {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
            {
                type = "sketch";
            }
            else
            { // U_SPIFFS
                type = "filesystem";
            }

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type); });
    }

    /**
     * @brief Connects the device to the WiFi network and the MQTT server.
     *
     * This function checks if the device is connected to the WiFi network and the MQTT server.
     * If the device is not connected to the WiFi network, it connects to the network.
     * If the device is not connected to the MQTT server, it connects to the server.
     */
    void connect()
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            connectWiFi();
        }
        if (!client.connected())
        {
            connectServer();
        }
    }

    /**
     * @brief Sends a message to the MQTT server indicating that the device is online.
     */
    void online()
    {
        if (currentMillis - prevTime >= onlineCycle || firstCycle)
        {
            prevTime = currentMillis;
            firstCycle = false;
            Serial.print(onlineId);
            Serial.print(" - ");
            Serial.println(String(prevTime).c_str());
            client.publish(onlineId, String(prevTime).c_str());
        }
    }

    /**
     * @brief Handles the message received from the MQTT server.
     *
     * This function handles the message received from the MQTT server.
     * It prints the topic and message to the serial monitor.
     *
     * @param topic The topic the message was received on.
     * @param payload The message payload.
     * @param length The length of the message payload.
     */
    void callback(char *topic, byte *payload, unsigned int length)
    {
        char p[length + 1] = {};
        for (int i = 0; i < length; i++)
        {
            p[i] = payload[i];
        }
        p[length] = '\0';

        msgPayload = new char[length + 1];
        std::copy(p, p + length + 1, msgPayload);

        if (verbose)
        {
            Serial.print(String(topic));
            Serial.print(" - ");
            Serial.println(String(msgPayload));
        }
    }

public:
    PubSubClient client{espClient}; // The PubSubClient object for MQTT communication.

    /**
     * @brief Constructs a NetworkManager object with the specified client name.
     *
     * @param clientID The name of the client.
     * @param password The password for OTA.
     * @param pin The pin to be used as an indicator of the connection status.
     * @param verbose The verbose mode flag.
     */
    NetworkManager(const char *clientID, const char *password, int pin, bool verbose = false)
        : clientID(clientID), password(password), pin(pin), verbose(verbose)
    {
        this->clientID = clientID;
        this->pin = pin;
        this->password = password;
        this->mac = WiFi.macAddress();

        Serial.begin(115200);
        pinMode(pin, OUTPUT);

        strcat(clientName, clientID);
        strcat(onlineId, clientID);

        Serial.println("Iniciando conexões WiFi e MQTT...");
        Serial.print("Device ID: \0");
        Serial.println(clientName);
        Serial.println("MAC Address: " + mac);

        client.setCallback([&](char *topic, byte *payload, unsigned int length)
                           { this->callback(topic, payload, length); });
    }

    /**
     * @brief Sets up the WiFi connection.
     *
     * This function sets up the WiFi connection with the specified SSID and password.
     *
     * @param netSSID The SSID of the network to connect to.
     * @param netPass The password of the network.
     */
    void setupWiFi(const char *netSSID, const char *netPass)
    {
        this->netSSID = netSSID;
        this->netPass = netPass;
    }

    /**
     * @brief Sets up the MQTT server connection.
     *
     * This function sets up the MQTT server connection with the specified server address, port, username, and password.
     *
     * @param mqttServer The MQTT server address.
     * @param mqttPort The MQTT server port.
     * @param mqttUser The MQTT username.
     * @param mqttPass The MQTT password.
     * @param topic The MQTT topic to subscribe to.
     */
    void setupMQTT(const char *mqttServer, int mqttPort, const char *mqttUser = "none", const char *mqttPass = "none", const char *topic = "none")
    {

        this->mqttServer = mqttServer;
        this->mqttPort = mqttPort;
        this->mqttUser = mqttUser;
        this->mqttPass = mqttPass;
        this->topic = topic;
    }

    /**
     * @brief Initializes the network connection and MQTT communication.
     */
    void begin()
    {
        startOTA();
        client.setServer(mqttServer, mqttPort);
    }

    /**
     * @brief Sets the verbose mode flag.
     *
     * @param verbose The verbose mode flag.
     */
    void setVerbose(bool verbose)
    {
        this->verbose = verbose;
    }

    /**
     * @brief Checks if the device is connected to the WiFi network and the MQTT server.
     *
     * @return True if the device is connected to the WiFi network and the MQTT server, false otherwise.
     */
    void isConnected()
    {
        if (WiFi.status() == WL_CONNECTED && client.connected())
        {
            return;
        }
        else
        {
            connect();
        }
    }

    /**
     * @brief Publishes a message to the specified topic on the MQTT server.
     *
     * @param topic The topic to publish the message to.
     * @param message The message to publish.
     */
    void publish(const char *topic, const char *message)
    {
        Serial.print(topic);
        Serial.print(" - ");
        Serial.println(message);
        client.publish(topic, message);
    }

    /**
     * @brief Handles the network connection and MQTT communication in the loop function.
     *
     * This function handles the network connection and MQTT communication in the loop function.
     * It checks for OTA updates, handles the MQTT communication, checks the connection status, and sends a message to the MQTT server indicating that the device is online.
     */
    void start()
    {
        currentMillis = millis();
        ArduinoOTA.handle();
        isConnected();
        if (verbose)
        {
            online();
        }
        if (currentMillis - prevCycle >= cycleTime || firstCycle)
        {
            prevCycle = currentMillis;
            deployCycle = true;
            firstCycle = false;
        }
        else
        {
            deployCycle = false;
        }
    }

    /**
     * @brief Sets the cycle time for the device to be online.
     *
     * @param cycleTime The cycle time in milliseconds.
     */
    void setCycleTime(unsigned long cycleTime)
    {
        this->cycleTime = cycleTime;
    }

    /**
     * @brief Sets the cycle time for the device to be online.
     *
     * @param onlineCycle The cycle time in milliseconds.
     */
    void setOnlineCycle(unsigned long onlineCycle)
    {
        this->onlineCycle = onlineCycle;
    }

    char *getPayload()
    {
        return msgPayload;
    }

    /**
     * @brief Checks if the device should run the cycle.
     *
     * @return True if the device should run the cycle, false otherwise.
     */
    bool runCycle()
    {
        return deployCycle;
    }

    /**
     * @brief Handles the MQTT communication in the loop function.
     *
     * This function handles the MQTT communication in the loop function.
     * It checks for OTA updates, handles the MQTT communication, and sends a message to the MQTT server indicating that the device is online.
     */
    void end()
    {
        client.loop();
    }
};