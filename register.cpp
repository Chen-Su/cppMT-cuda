#include "register.h"

int get_mac(char * mac, int len_limit)    //返回值是实际写入char * mac的字符个数（不包括'\0'）
{
	struct ifreq ifreq;
	int sock;

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("socket");
		return -1;
	}
	strcpy(ifreq.ifr_name, "wlan0");    //Currently, only get eth0

	if (ioctl(sock, SIOCGIFHWADDR, &ifreq) < 0)
	{
		perror("ioctl");
		return -1;
	}


	return snprintf(mac, len_limit, "%X:%X:%X:%X:%X:%X", (unsigned char)ifreq.ifr_hwaddr.sa_data[0], (unsigned char)ifreq.ifr_hwaddr.sa_data[1], (unsigned char)ifreq.ifr_hwaddr.sa_data[2], (unsigned char)ifreq.ifr_hwaddr.sa_data[3], (unsigned char)ifreq.ifr_hwaddr.sa_data[4], (unsigned char)ifreq.ifr_hwaddr.sa_data[5]);
}

int trackerRegister()
{
	char    port_buf;
	char    szMac[18];
	int     nRtn = get_mac(szMac, sizeof(szMac));

	if (nRtn < 1)
	{
		std::cout << "wrong machine type" << std::endl;
		exit(0);
	}


	char mac_adress[13];
	int j = 0;
	int count_num = 0;
	int i;

	for (i = 0; i<13; i++)
	{
		mac_adress[i] = '\0';
	}

	for (i = 0; i<18; i++)
	{
		if (szMac[i] == '\0')
		{
			if (count_num == 1)
			{
				mac_adress[j] = '0';
				j++;
				mac_adress[j] = szMac[i - 1];
				j++;
			}
			else
			{
				mac_adress[j] = szMac[i - 2];
				j++;
				mac_adress[j] = szMac[i - 1];
				j++;
			}
			break;
		}

		if (szMac[i] == ':')
		{
			if (count_num == 1)
			{
				mac_adress[j] = '0';
				j++;
				mac_adress[j] = szMac[i - 1];
				j++;
			}
			else
			{
				mac_adress[j] = szMac[i - 2];
				j++;
				mac_adress[j] = szMac[i - 1];
				j++;
			}
			count_num = 0;
		}
		else
			count_num++;
	}



	//register
	int shift_num = 0;
	int insert_num = 0;
	shift_num = 4;
	insert_num = 13;

	bool registYN = false;

	char serial_num[15];
	//char true_serial_num[13];

	for (int i = 0; i < 15; i++)
	{
		serial_num[i] = '\0';
	}

	if (insert_num >= 10)
	{
		serial_num[0] = insert_num + 55;
	}
	else
		serial_num[0] = insert_num + 48;

	serial_num[insert_num] = shift_num + 48;

	j = 0;
	for (int i = 1; i < 14; i++)
	{
		if (serial_num[i] == '\0')
		{
			serial_num[i] = mac_adress[j] + shift_num;
			if (serial_num[i]>57 && serial_num[i]<65)
			{
				serial_num[i] += 7;
			}
			j++;
		}
	}

	while (!registYN)
	{
		char buf123[128];
		if (!sha1_hash(serial_num, buf123))
		{
			std::cout << "register fail¡" << std::endl;
			return 0;
		}

		char hashread[128];
		for (int i = 0; i < 128; i++)
		{
			hashread[i] = '\0';
		}

		std::ifstream in("key.txt");

		if (!in)
		{
			in.close();
			std::cout << "need register." << std::endl;
			std::cout << "your machine id: " << serial_num << std::endl;
			std::cout << "input your cdkey: ";
			std::string registNum;
			std::cin >> registNum;
			std::ofstream out("key.txt");
			out << registNum;
			out.close();

			continue;

		}
		


		int keynum = 0;
		while (in.peek() != EOF)
		{
			hashread[keynum] = in.get();
			keynum++;
		}

		in.close();


		if (strcmp(hashread, buf123) != 0)
		{
			std::cout << "need register." << std::endl;
			std::cout << "your machine id: " << serial_num << std::endl;
			std::cout << "input your cdkey: ";
			std::string registNum;
			std::cin >> registNum;
			std::ofstream out("key.txt", std::ios::trunc);
			out << registNum;
			out.close();

		}
		else
		{
			std::cout << "pass!!" << std::endl;
			registYN = true;
		}
	}
}
