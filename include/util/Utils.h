#ifndef UTILS_H
#define UTILS_H

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
namespace easytracker {
	class Utils
	{
	public:
		static std::vector<std::string> stringSplit(std::string &line, char splitchar) {
			std::string buff{ "" };
			std::vector<std::string>strs;
			for (auto n : line) {
				if (n != splitchar)
					buff += n;
				else if (n == splitchar && buff != "")
				{
					strs.push_back(buff);
					buff = "";
				}
			}
			if (buff != "")
			{
				strs.push_back(buff);
			}
			return strs;
		}

		static std::vector<std::string> loadObjectsNames(std::string& filename) {
			std::ifstream file(filename);
			std::vector<std::string> file_lines;
			if (!file.is_open()) return file_lines;
			for (std::string line; getline(file, line);) file_lines.push_back(line);
			std::cout << "object names loaded \n";
			return file_lines;
		}
	};

}
#endif // !UTILS_H


