/**
 * ESP32 secrets file
 * 
 * Contains information required to connect to WiFi and in-turn, AWS IoT
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */


/*
  Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
  Permission is hereby granted, free of charge, to any person obtaining a copy of this
  software and associated documentation files (the "Software"), to deal in the Software
  without restriction, including without limitation the rights to use, copy, modify,
  merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <pgmspace.h>

#define SECRET
#define THINGNAME "MyRobot"

const char WIFI_SSID[] = "IoT4";
const char WIFI_PASSWORD[] = "123456789";
const char AWS_IOT_ENDPOINT[] = "a6y3ms0k7agqq-ats.iot.us-east-1.amazonaws.com";

/* Amazon Root CA 1 */
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

/* Device Certificate */
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVAPqkAOTuDmwZHfBa9wy8RdKybwiDMA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yMzAxMTAxNzE2
MjdaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDKvD33gONKLOf3mcuD
JPb/YsgUdML55DlVYOZCzyZX6RW+eePDuewhH+kkvqfvuRxOgZxzj8RGLBxkTLXo
ZDzmy6Z27qsUaSIAhhr6TAEuL6Piaug/UNhCsB702vXrrUrSOWbwVOdecmcW6qn5
gswcQcPQ0Km44aIlxwvU14cTobAuKiOhcyz2NhdvVXl13wdi59NAC9cGVWtBKhhD
FPTib09vz/tK5irigZ7ejkGn003udh/eFtXcNzviFLymw4ERDYYT6smxXpfGxpFm
NIYSoSqKA6lVVuzpdYkanNAiU9CvLdaODll9kQbGbbQm5PIZ879TLd3V2bI4CM09
IIk/AgMBAAGjYDBeMB8GA1UdIwQYMBaAFBQItyW5BOqQzf/jMElI0EYhWgR0MB0G
A1UdDgQWBBTzWNAMSFMfo1VCwZslL6/sQWgm/TAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAdtaUbETVNfn9DIGJYPc4p/vS
JX63GO7tNUU8VOfBsaKtIhyKLPRF4eQnxd7MZYfZwyBGMFQLJazI/nkrgIrjZaw2
imhaJ8QI9FYJHz2/hwd87eG2wEy9oxaI2Fc+3WU8Lyx4xBc1R1gtNXx82wkU+erD
UZQVlD5NyyiNKGOupVBbIIaBYrIVubTq2lWJlHz/LtC1or0nbvGEn/hjvMmotzPY
PdjERVxKgXN5ZmXeqV1DQN0gjlMKaj7e0Yz6LePYXWv6sFbkNTe6BZ7HeNrVcMmM
wFdgnZav5LSgromxgnloLt7XWY+nypGXsoqzm58z6gmcLOgn96VtG2oRVqQkVg==
-----END CERTIFICATE-----
)KEY";

/* Device Private Key */
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAyrw994DjSizn95nLgyT2/2LIFHTC+eQ5VWDmQs8mV+kVvnnj
w7nsIR/pJL6n77kcToGcc4/ERiwcZEy16GQ85sumdu6rFGkiAIYa+kwBLi+j4mro
P1DYQrAe9Nr1661K0jlm8FTnXnJnFuqp+YLMHEHD0NCpuOGiJccL1NeHE6GwLioj
oXMs9jYXb1V5dd8HYufTQAvXBlVrQSoYQxT04m9Pb8/7SuYq4oGe3o5Bp9NN7nYf
3hbV3Dc74hS8psOBEQ2GE+rJsV6XxsaRZjSGEqEqigOpVVbs6XWJGpzQIlPQry3W
jg5ZfZEGxm20JuTyGfO/Uy3d1dmyOAjNPSCJPwIDAQABAoIBAHDwTuD7ZyMiQxNX
FN15ETQSNn3W0etd7WaXtY6QUb71dQyIYI7fmyCU4095+pn8GD3PzHVAKMoitqpV
ZZ0Rgi/cUV699IZJTwzVqF+5SYsONRDkF392LRNxg71J4WeqQR09pw/JbJ7bwnws
vAXojIZNSKWiKHp7D9ZVvUyTyIbxNP0Fr+0sn/oupjHFyD4AYfNFt3OHmR0x2c5d
d6dRuR/8tDOYM5FNgdLZmpS6wrIX0HvjTibPyByYUaxlTDY6xWuK6oCSZ86px4qO
WU3jMAOp0VbqfJAecAmKKxxuyBQkJsd/t4khVca3ROSyCjsB7vnoSYrTVQWjNuwj
+oKXilkCgYEA6zbE2IrJM1kijM+evzWe84BNODAXWFFxv1QLHTSt2mp5rcxOeybe
LRUuciznubRGDh3tzdzpv2itBNLhgXh0bSbS8OEwWzvs9jDRKBMG22sQUC07iuGu
pRFVAnw6I/WcFHhHlgW+0Soa+IUqLWkFT9GQkncK0cYMt5fQ+vaSTTsCgYEA3Ka2
ElISkIT/9iDhpdAGv4bOOQa4ToPI5l6YA6BasGhWiRZzQ96MkNrFVyf2CWvhblHu
0GM0NJRBNinxqHHZKr5lCG+7p+oTrSc2XUFy3IZUjM4N5hJBCtVQVf5UuqtBDRmX
ybOhq+BYZ6E3ThPfzWS6WhYZRgQPnek9zEwjA80CgYBqfW5bd2ImPDm3NJxzhyxC
pBjR0nwPQThVXhB2FY6Mb15NgE0b1noKQSe5C3mSCvsYkxZB+L1fKl4C7BKqkkVr
0YL5CYriE1xyDaH0GcEl/+/vzLrDYu+1zgYFzQZpl14lqlsGoe1FN2HkmC2DR6Pf
AksI9JpSZO8HA9XaeV29VwKBgDi1E7fzpdfEhq/v1W4y33kboSZgxr9O+TPTfcf4
zzXlKtBCl88KhB+6SdtTZtUXB7G9lt2xkTouG/BaqUO8Nq7Yjci0RosqCnKG0F75
Mhbg4xvo3QOTD2pEVkgze/0ZhDdkq3DoHJW6q52+fiMv0mXMDGSYp1gqNQVFa190
9c7FAoGBAKx2Lo3dSIPUDzWZYGS13ghaQalNrCGJTn35AyIg47AgWM6eddEDwRNY
wxMSnH0x97Rm5udUhZGJOpO58Fj70dZWKgfBBSSUp/EGwTF235c94S7qQdqPBzod
mYtz05uh007TKvrbm8ZJTP+TOCPDUclYFTozI8AgbvCTIuhs/uIi
-----END RSA PRIVATE KEY-----
)KEY";