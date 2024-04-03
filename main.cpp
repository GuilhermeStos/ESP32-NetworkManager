#include "network.cpp" // include network, OTA and MQTT manager

const char *id = "i001"; // device id

NetworkManager net(id, "pass", 2, true); // network and MQTT manager instance with id and a LED pin

void setup()
{
    net.setupWiFi("your_network_ssid", "your_network_pass"); // create a bridge to connect to the MQTT broker
    net.setupMQTT("your_broker_ip", 1883);                   // setup MQTT broker. You can add USERNAME, PASSWORD and a TOPIC to subscribe too
    net.begin();                                             // start the network manager
}

void loop()
{
    net.start();            // start the network manager
    net.setCycleTime(5000); // set the cycle time to 5 seconds

    char *payload = net.getPayload(); // get the payload from the MQTT broker

    if (net.runCycle()) // run the cycle
    {
        net.publish("test", payload); // publish the payload to the MQTT broker
    }
    net.end(); // end the network manager
}
