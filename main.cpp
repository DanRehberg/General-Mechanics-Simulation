#include <iostream>
#include <string>
#include <exception>
#include <SDL.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <chrono>
#include <fstream>
#include "ngon.hpp"
#include "shaders.hpp"

SDL_GLContext glContext;
SDL_Window* window = NULL;
constexpr char windowName[] = "Shock Propagation Test";

void buildLineBuffers();
void closeLineBuffers();
void buildShaders();
void closeShaders();
void closeSDL();
void initSDL();

GLuint vert = 0, frag = 0, program = 0;
GLint uniformIndex = -1, colorIndex = -1;
GLuint vbo = 0, vao = 0, ubo = 0;
long long tickSum = 0;
constexpr bool reportData = true;
constexpr size_t ignoreInitialCount = 450;

int main(int argc, char* argv[])
{
	char yesNo = ' ';
	std::cout << "Shock Test: ";
	std::cin >> yesNo;
	if (yesNo == 'n' || yesNo == 'N')
	{
		int quantity = 0;
		std::cout << "NGon size?";
		int badInput = 0;
		while (quantity < 3 || quantity >= 100)
		{
			std::cout << "\nValue must be 3 or greater, but less than 100: ";
			std::cin >> quantity;
			if (std::cin.fail())
			{
				std::cerr << "Bad input value\n";
				std::cin.clear();
				std::string clear;
				std::getline(std::cin, clear);
				if (++badInput >= 5)return -1;
			}
		}

		bool quit = false;

		try
		{
			initSDL();
		}
		catch (std::exception err)
		{
			std::cout << err.what() << std::endl;
			quit = true;
		}

		buildShaders();
		buildLineBuffers();

		SDL_Event sdlEvent;

		float pos[] = { 0.0f, 0.25f, 0.0f, 1000.0f, 1000.0f, 1.0f };
		float color[] = { 1.0f, 1.0f, 1.0f };
		std::chrono::time_point<std::chrono::steady_clock> startTime, endTime;
		try
		{
			NGon ground(glm::vec3(960.0f, -900.0f, 0.0f), 959.0f, 4, 0.0f);
			NGon wallLeft(glm::vec3(-500.0f, 0.0f, 0.0f), 539.0f, 4, 0.0f);
			NGon wallRight(glm::vec3(2400.0f, 0.0f, 0.0f), 539.0f, 4, 0.0f);
			ground.setColor(glm::vec3(1.0f, 1.0f, 1.0f));
			wallLeft.setColor(glm::vec3(1.0f, 1.0f, 1.0f));
			wallRight.setColor(glm::vec3(1.0f, 1.0f, 1.0f));
			
			std::vector<NGon> objects;
			objects.resize(1000);
			for (size_t i = 0; i < 100; ++i)
			{
				for (size_t j = i * 10; j < i * 10 + 10; ++j)
				{
					glm::vec3 pos(80.0f + static_cast<float>((j - i * 10) * 150), static_cast<float>(i) * 50.0f + 400.0f, 0.0f);
					objects[j] = NGon(pos, 15, quantity, 5.0f);
					objects[j].setColor(glm::vec3(1.0f, 1.0f, 0.0f));
				}
			}

			/*for (size_t i = 0; i < objects.size() / 4; ++i)
			{
				glm::vec3 pos(80.0f + static_cast<float>(i * 150), 540.0f, 0.0f);
				objects[i] = NGon(pos, 50, quantity, 5.0f);
				objects[i].setColor(glm::vec3(1.0f, 1.0f, 0.0f));
			}
			for (size_t i = objects.size() / 4; i < objects.size() / 2; ++i)
			{
				glm::vec3 pos(80.0f + static_cast<float>((i - objects.size() / 4) * 150), 340.0f, 0.0f);
				objects[i] = NGon(pos, 50, quantity, 5.0f);
				objects[i].setColor(glm::vec3(1.0f, 1.0f, 0.0f));
			}
			for (size_t i = objects.size() / 2; i < objects.size() / 2 + objects.size() / 4; ++i)
			{
				glm::vec3 pos(80.0f + static_cast<float>((i - objects.size() / 2) * 150), 140.0f, 0.0f);
				objects[i] = NGon(pos, 50, quantity, 5.0f);
				objects[i].setColor(glm::vec3(1.0f, 1.0f, 0.0f));
			}
			for (size_t i = objects.size() / 2 + objects.size() / 4; i < objects.size(); ++i)
			{
				glm::vec3 pos(80.0f + static_cast<float>((i - objects.size() / 2 - objects.size() / 4) * 150), 640.0f, 0.0f);
				objects[i] = NGon(pos, 50, quantity, 5.0f);
				objects[i].setColor(glm::vec3(1.0f, 1.0f, 0.0f));
			}*/

			while (!quit)
			{

			

				startTime = std::chrono::steady_clock::now();
				//Event Polling
				while (SDL_PollEvent(&sdlEvent) != 0)
				{
					switch (sdlEvent.type)
					{
					case SDL_KEYDOWN: quit = (sdlEvent.key.keysym.scancode == 41) ? true : false; break;
					default: break;
					}
				}
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				//Recurring Trial Area
				{
					if (tickSum > Simulation::updateInterval)
					{
						//Time integration
						tickSum = 0;
						for (size_t i = 0; i < objects.size(); ++i)
						{
							objects[i].update(Simulation::deltaTime);
							Collision::simulatePair(objects[i], wallLeft);
							Collision::simulatePair(objects[i], wallRight);
							Collision::simulatePair(objects[i], ground);
							Collision::selfConstraints(objects[i]);
							glm::vec3 v = objects[i].getVelocity();
							v *= (glm::length(v) / 20.0f);
							v += 1.f;
							//v.x = v.y;
							//v.z = temp + v.y * 0.4;
							//v.z = temp;
							//v.y = temp;
							//v.x = v.y;

							v *= 0.5f;
							objects[i].setColor(v);
						}
						for (size_t i = 0; i < objects.size(); ++i)
						{
							for (size_t j = i + 1; j < objects.size(); ++j)
							{
								Collision::simulatePair(objects[i], objects[j]);
							}
						}
						Collision::solveConstraints();
					}

					glBindVertexArray(vao);
					glUseProgram(program);
					for (size_t i = 0; i < objects.size(); ++i)
					{
						objects[i].getColor(color);
						for (size_t j = 0; j < objects[i].getN(); ++j)
						{
							objects[i].getLine(pos, j);
							glUniform3fv(uniformIndex, 2, &pos[0]);
							glUniform3fv(colorIndex, 1, &color[0]);
							glDrawArrays(GL_LINES, 0, 2);
						}
					}
					wallLeft.getColor(color);
					for (size_t i = 0; i < wallLeft.getN(); ++i)
					{
						wallLeft.getLine(pos, i);
						glUniform3fv(uniformIndex, 2, &pos[0]);
						glUniform3fv(colorIndex, 1, &color[0]);
						glDrawArrays(GL_LINES, 0, 2);
					}
					wallRight.getColor(color);
					for (size_t i = 0; i < wallRight.getN(); ++i)
					{
						wallRight.getLine(pos, i);
						glUniform3fv(uniformIndex, 2, &pos[0]);
						glUniform3fv(colorIndex, 1, &color[0]);
						glDrawArrays(GL_LINES, 0, 2);
					}
					ground.getColor(color);
					for (size_t i = 0; i < ground.getN(); ++i)
					{
						ground.getLine(pos, i);
						glUniform3fv(uniformIndex, 2, &pos[0]);
						glUniform3fv(colorIndex, 1, &color[0]);
						glDrawArrays(GL_LINES, 0, 2);
					}
					glBindVertexArray(0);
					glUseProgram(0);
				}
				//General Frame Updates
				SDL_GL_SwapWindow(window);
				endTime = std::chrono::steady_clock::now();
				tickSum += std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
			}
		}
		catch (std::exception err)
		{
			std::cout << err.what() << std::endl;
		}
	}
	else
	{
		int quantity = 0;
		std::cout << "NGon size?";
		int badInput = 0;
		while (quantity < 3 || quantity >= 100)
		{
			std::cout << "\nValue must be 3 or greater, but less than 100: ";
			std::cin >> quantity;
			if (std::cin.fail())
			{
				std::cerr << "Bad input value\n";
				std::cin.clear();
				std::string clear;
				std::getline(std::cin, clear);
				if (++badInput >= 5)return -1;
			}
		}

		//Iteration and Data Collection variables
		int inputIndex = -1, frameCount = 0, currentFrame = 0;
		std::vector<std::vector<float>> areaDeviations;
		std::vector<std::vector<glm::vec3>> positionDeviations;
		std::vector<float> allAreaAverage, allPosDistanceAverage, allAreaStd, allPosStd;

		badInput = 0;
		std::cout << "How many frames to run per trial?";
		while (frameCount < 1 || frameCount >= 2000)
		{
			std::cout << "\nValue must be 1 or greater, but less than 2,000: ";
			std::cin >> frameCount;
			if (std::cin.fail())
			{
				std::cerr << "Bad input value\n";
				std::cin.clear();
				std::string clear;
				std::getline(std::cin, clear);
				if (++badInput >= 5)return -1;
			}
		}
		currentFrame = frameCount;

		bool quit = false;

		try
		{
			initSDL();
		}
		catch (std::exception err)
		{
			std::cout << err.what() << std::endl;
			quit = true;
		}

		buildShaders();
		buildLineBuffers();

		SDL_Event sdlEvent;

		float pos[] = { 0.0f, 0.25f, 0.0f, 1000.0f, 1000.0f, 1.0f };
		float color[] = { 1.0f, 1.0f, 1.0f };
		std::chrono::time_point<std::chrono::steady_clock> startTime, endTime;
		try
		{
			NGon n(glm::vec3(960.0f, 540.0f, 0.0f), 100.0f, quantity, 5.0f);
			float expectedAverage = 0.f;
			for (size_t i = 0; i < quantity; ++i)
			{
				expectedAverage += n.getArea(i);
			}
			expectedAverage /= static_cast<float>(quantity);
			std::cout << "Expected average area: " << expectedAverage << "\n";
			n.setColor(glm::vec3(0.0f, 1.0f, 0.0f));
			NGon ground(glm::vec3(960.0f, -900.0f, 0.0f), 959.0f, 4, 0.0f);
			ground.setColor(glm::vec3(0.0f, 1.0f, 0.0f));
			NGon another(glm::vec3(960.0f, 220.0f, 0.0f), 50.0f, 4, 5.0f);
			std::vector<glm::vec3> rootForInputSpaceOrder;
			std::cout << "n: " << n.getID() << " ground: " << ground.getID() << " another: " << another.getID() << '\n';
			for (size_t i = 0; i < quantity; ++i)
			{
				glm::vec3 direction = n.getParticle(i) - n.getPosition();
				rootForInputSpaceOrder.push_back(glm::normalize(direction));
			}

			while (!quit)
			{

				if (currentFrame >= frameCount)
				{
					//Analysis
					float averagePosDev = 0.f, averageAreaDev = 0.f;
					float stdPos = 0.f, stdArea = 0.f;
					std::vector<float> positionDist(positionDeviations.size());
					for (size_t i = 0; i < positionDeviations.size(); ++i)
					{
						for (size_t j = 0; j < positionDeviations[i].size(); ++j)
						{
							averageAreaDev += areaDeviations[i][j];
							if (i < positionDeviations.size() - 1)
							{
								float mag = glm::length(positionDeviations[i + 1][j]
									- positionDeviations[i][j]);
								averagePosDev += mag;
								positionDist.push_back(mag);
							}
						}
					}
					if (positionDeviations.size() > 0)
					{
						float posN = positionDeviations[0].size();
						float areaN = areaDeviations.size() * areaDeviations[0].size();
						averagePosDev /= posN;
						averageAreaDev /= areaN;
						float sumP = 0.f, sumA = 0.f;
						for (size_t i = 0; i < positionDist.size(); ++i)
						{
							float val = positionDist[i] - averagePosDev;
							sumP += val * val;
						}
						for (size_t i = 0; i < areaDeviations.size(); ++i)
						{
							for (size_t j = 0; j < areaDeviations[i].size(); ++j)
							{
								float val = areaDeviations[i][j] - averageAreaDev;
								sumA += val * val;
							}
						}
						stdPos = std::sqrt(sumP / posN);
						stdArea = std::sqrt(sumA / areaN);
						allAreaAverage.push_back(averageAreaDev);
						allAreaStd.push_back(stdArea);
						allPosDistanceAverage.push_back(averagePosDev);
						allPosStd.push_back(stdPos);
					}
					//reset to next trial
					currentFrame = 0;
					if (++inputIndex == quantity)
					{
						std::string fileName = "No_Record" + std::to_string(quantity) + "-NGon_" +
							std::to_string(frameCount) + "-Steps_" +
							std::to_string(Simulation::solverIterations) + "-Iterations_" +
							((Simulation::verletResolution == true) ? "Verlet.csv" : "Lagrange.csv");
						if (reportData)
						{
							std::ofstream out(fileName);
							if (out.is_open())
							{
								std::cout << rootForInputSpaceOrder.size() << " " <<
									allPosDistanceAverage.size() << " " <<
									allPosStd.size() << " " <<
									allAreaAverage.size() << " " <<
									allAreaStd.size() << std::endl;
								out << "Dir X, Dir Y, Dir Z, Avg Pos, Std Pos, Avg Area, Std Area";
								for (size_t i = 0; i < quantity; ++i)
								{
									out << '\n' << rootForInputSpaceOrder[i].x << ',' <<
										rootForInputSpaceOrder[i].y << ',' <<
										rootForInputSpaceOrder[i].z << ',' <<
										allPosDistanceAverage[i] << ',' <<
										allPosStd[i] << ',' <<
										allAreaAverage[i] << ',' <<
										allAreaStd[i];
								}
								out.close();
							}
						}
						quit = true;
						continue;
					}
					n = NGon(glm::vec3(960.0f, 540.0f, 0.0f), 100.0f, quantity, 5.0f);
					Simulation::shockDirection = rootForInputSpaceOrder[inputIndex];
					areaDeviations.clear();
					positionDeviations.clear();
					std::cout << "trial: " << inputIndex << " dir: " << Simulation::shockDirection.x << " " << Simulation::shockDirection.y << "\n";
				}

				startTime = std::chrono::steady_clock::now();
				//Event Polling
				while (SDL_PollEvent(&sdlEvent) != 0)
				{
					switch (sdlEvent.type)
					{
					case SDL_KEYDOWN: quit = (sdlEvent.key.keysym.scancode == 41) ? true : false; break;
					default: break;
					}
				}
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				//Recurring Trial Area
				{
					if (tickSum > Simulation::updateInterval)
					{
						if (currentFrame >= ignoreInitialCount)
						{
							areaDeviations.push_back(std::vector<float>());
							positionDeviations.push_back(std::vector<glm::vec3>());
							//Collect data at start of frame
							for (size_t i = 0; i < n.getN(); ++i)
							{
								positionDeviations[currentFrame - ignoreInitialCount].push_back(n.getParticle(i));
								areaDeviations[currentFrame - ignoreInitialCount].push_back(n.getArea(i));
							}
						}

						//Time integration
						n.update(Simulation::deltaTime);
						//another.update(Simulation::deltaTime);
						tickSum = 0;
						++currentFrame;

						Collision::simulatePair(n, ground);
						//Collision::simulatePair(another, ground);
						//Collision::simulatePair(n, another);
						//Collision::simulatePair(another, n);
						//Collision::simulatePair(ground, another);
						//Collision::simulatePair(ground, n);
						Collision::selfConstraints(n);
						//Collision::selfConstraints(another);
						Collision::solveConstraints();
					}

					glBindVertexArray(vao);
					glUseProgram(program);
					n.getColor(color);
					for (size_t i = 0; i < n.getN(); ++i)
					{
						n.getLine(pos, i);
						glUniform3fv(uniformIndex, 2, &pos[0]);
						glUniform3fv(colorIndex, 1, &color[0]);
						glDrawArrays(GL_LINES, 0, 2);
					}
					ground.getColor(color);
					for (size_t i = 0; i < ground.getN(); ++i)
					{
						ground.getLine(pos, i);
						glUniform3fv(uniformIndex, 2, &pos[0]);
						glUniform3fv(colorIndex, 1, &color[0]);
						glDrawArrays(GL_LINES, 0, 2);
					}
					/*for (size_t i = 0; i < another.getN(); ++i)
					{
						another.getLine(pos, i);
						glUniform3fv(uniformIndex, 2, &pos[0]);
						glUniform3fv(colorIndex, 1, &color[0]);
						glDrawArrays(GL_LINES, 0, 2);
					}*/
					glBindVertexArray(0);
					glUseProgram(0);
				}
				//General Frame Updates
				SDL_GL_SwapWindow(window);
				endTime = std::chrono::steady_clock::now();
				tickSum += std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();
			}
		}
		catch (std::exception err)
		{
			std::cout << err.what() << std::endl;
		}
	}

	closeShaders();
	closeLineBuffers();
	closeSDL();

	return 0;
}

void buildLineBuffers()
{
	GLuint lines[] = { 0, 1 };
	glGenVertexArrays(1, &vao);
	glGenBuffers(1, &vbo);
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(lines), lines, GL_STATIC_DRAW);
	glVertexAttribIPointer(0, 1, GL_UNSIGNED_INT, sizeof(GLuint), (void*)0);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	GLenum err;
	while ((err = glGetError()) != GL_NO_ERROR)
	{
		std::cout << "vao errs present: " << err << '\n';
	}
}

void buildShaders()
{
	vert = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vert, 1, &vertShader, NULL);

	glCompileShader(vert);
	// check for shader compile errors
	int success;
	char infoLog[512];
	glGetShaderiv(vert, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vert, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
	}

	frag = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(frag, 1, &fragShader, NULL);

	glCompileShader(frag);
	// check for shader compile errors
	glGetShaderiv(frag, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(frag, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
	}

	program = glCreateProgram();
	glAttachShader(program, vert);
	glAttachShader(program, frag);
	glLinkProgram(program);
	glGetProgramiv(program, GL_LINK_STATUS, &success);
	if (!success)
	{
		glGetProgramInfoLog(program, 512, NULL, infoLog);
		std::cout << "linking failed: " << infoLog << std::endl;
	}

	glDeleteShader(vert);
	glDeleteShader(frag);

	glUseProgram(0);
	uniformIndex = glGetUniformLocation(program, "positions");
	colorIndex = glGetUniformLocation(program, "color");

	glGenBuffers(1, &ubo);
	glBindBuffer(GL_UNIFORM_BUFFER, ubo);
	glBufferData(GL_UNIFORM_BUFFER, sizeof(glm::mat4), NULL, GL_STATIC_DRAW);
	glBindBuffer(GL_UNIFORM_BUFFER, 0);
	glBindBufferRange(GL_UNIFORM_BUFFER, 0, ubo, 0, sizeof(glm::mat4));

	glm::mat4 ortho = glm::ortho(0.0f, 1920.0f, 0.0f, 1080.0f);
	glBindBuffer(GL_UNIFORM_BUFFER, ubo);
	glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(glm::mat4), glm::value_ptr(ortho));
	glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void closeLineBuffers()
{
	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(1, &vbo);
	vao = 0;
	vbo = 0;
}

void closeShaders()
{
	glDeleteProgram(program);
	program = 0;
}

void closeSDL()
{
	if (window != NULL)
	{
		SDL_DestroyWindow(window);
		window = NULL;
	}
	if (glContext != NULL)
	{
		SDL_GL_DeleteContext(glContext);
	}
}

void initSDL()
{
	if (SDL_Init(SDL_INIT_VIDEO) != 0)
	{
		throw std::exception("Initialization of SDL with video failed.");
	}
	else
	{
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
		SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 6);
		SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
		SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
		window = SDL_CreateWindow(
			windowName,
			SDL_WINDOWPOS_UNDEFINED,
			SDL_WINDOWPOS_UNDEFINED,
			1920,
			1080,
			SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
		if (window == NULL)
		{
			throw std::exception("Window could not be initialized.");
		}
		glContext = SDL_GL_CreateContext(window);
		if (glContext == NULL)
		{
			throw std::exception("OpenGL context could not be initialized.");
		}
		SDL_GL_SetSwapInterval(0);
		glewExperimental = GL_TRUE;
		GLenum glewerror = glewInit();
		if (glewerror != GLEW_OK)
		{
			throw std::exception("GLEW could not be initialized.");
		}
		SDL_GL_MakeCurrent(window, glContext);

		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClearDepth(1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glLineWidth(3.0f);

		GLenum err;
		while ((err = glGetError()) != GL_NO_ERROR)
		{
			std::cout << "initial errs present: " << err << '\n';
		}
	}
}
