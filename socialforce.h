// header file: implementation for Social Force Model

// include the potentials for pedestrain to pedestrain
//, pedestrain to space and pedestrain to acctract
#include "potentials.h" 
#include <math.h>

#define PI acos(-1) // define the pi number

// class: implement the social force model
class socialforcemodel {
public:
	// Initialize the constructor with input initial states
	socialforcemodel(std::vector<std::vector<double>> initial_states) : m_states(initial_states), 
		m_speeds(speeds(initial_states)), m_disired_direct(desired_directions(initial_states)) {};
	// with extra potentials without ped ped potential
	socialforcemodel(std::vector<std::vector<double>> initial_states,
		std::vector<std::vector<double>> spaces) : m_states(initial_states), m_spaces(spaces), 
		m_speeds(speeds(initial_states)), m_disired_direct(desired_directions(initial_states)) {};
	// with extra potentials
	/*socialforcemodel(std::vector<std::vector<double>> initial_states, ped_ped_potential ped_ped,
		ped_space_potential ped_space, ped_attract_potential ped_attract) : m_states(initial_states)
		, m_ped_space(&ped_space), m_ped_attract(&ped_attract) {};*/
	// ------------------------------------------
	// return: 2d vector for next states
	std::vector<std::vector<double>>socialforce_simulator();
	// ------------------------------------------
private:
	// input date with type (position x,position y,speed x, speed y
	//                       , destination x, destination y
	//                       , relaxation time tau [optional])
	std::vector<std::vector<double>> m_states; 
	std::vector<std::vector<double>> m_spaces;
	std::vector<std::vector<double>> m_disired_direct;
	std::vector<double> m_speeds;
	//-------------------------------------------
	//-------------------------------------------
	// initialize some parameters in paper
	double m_delta_t = 0.4;
	double m_max_speed_multiplier = 1.3;
	double m_tau = 0.5;
	double m_influence = 0.5;
	double m_cosphi = cos(200.0/2.0/180.0*PI);
	//-------------------------------------------
	// take the positions and destinations corresponding from states, 
	//    then compute with normalizing and return disired directions
	// return: 2d vector for disired directions
	std::vector<std::vector<double>> desired_directions(std::vector<std::vector<double>>);
	//-------------------------------------------
	// take the speeds corresponding from states, then normalize and return the speeds
	// return: 1d vector for speeds
	std::vector<double> speeds(std::vector<std::vector<double>>);
	//-------------------------------------------
	// determine that is obstacles or pedestrains in sight  
	// return: 2d vector for is in sight or not
	std::vector< std::vector<std::vector<double>>> in_sight(std::vector<std::vector<std::vector<double>>>, std::vector<std::vector<double>>);
	//-------------------------------------------


};

std::vector< std::vector<std::vector<double>>> socialforcemodel::in_sight(std::vector<std::vector<std::vector<double>>> f_ab, std::vector<std::vector<double>>e) {
	unsigned int i, j;
	double temp_dot, temp_norm_cos;
	std::vector<std::vector<double>> w(e.size(), std::vector<double>(e.size()));
	for (i = 0; i < e.size(); i++) {
		for (j = 0; j < e.size(); j++) {
			temp_dot = (e[i][0] * -f_ab[i][j][0] + e[i][1] *-f_ab[i][j][1]);
			temp_norm_cos = sqrt(pow(f_ab[i][j][0], 2) + pow(f_ab[i][j][1], 2)) * m_cosphi;
			if (temp_dot < temp_norm_cos) {
				f_ab[i][j][0] = m_influence * f_ab[i][j][0];
				f_ab[i][j][1] = m_influence * f_ab[i][j][1];
			}
			
			//std::cout << f_ab[i][j][0] << " " <<  f_ab[i][j][1] << "\n";
		}
		//std::cout << "\n";
	}
	return f_ab;
}


std::vector<std::vector<double>> socialforcemodel::desired_directions(std::vector<std::vector<double>> states) {
	unsigned int i;
	double x, y, n;
	std::vector<std::vector<double>> directions(states.size());
	for (i = 0; i < states.size(); i++) {
		x = states[i][4] - states[i][0];
		y = states[i][5] - states[i][1];
		n = sqrt(pow(x, 2) + pow(y, 2));
		if (n != 0) {
			x /= n;
			y /= n;
		}
		else {
			x = 0;
			y = 0;
		}
		directions[i].push_back(x);
		directions[i].push_back(y);
	}
	return directions;
}

std::vector<double> socialforcemodel::speeds(std::vector<std::vector<double>> states) {
	unsigned int i;
	double x, y, n;
	std::vector<double> speed;
	for (i = 0; i < states.size(); i++) {
		x = states[i][2];
		y = states[i][3];
		n = sqrt(pow(x, 2) + pow(y, 2));
		speed.push_back(n);
	}
	return speed;
}

std::vector<std::vector<double>> socialforcemodel::socialforce_simulator() {

	// initialize the potential objects if no use extra input
	ped_ped_potential temp_ped_ped(m_delta_t);


	//m_disired_direct = desired_directions();
	//m_speeds = speeds();

	std::vector<std::vector<std::vector<double>>>f_ab;
	
	std::vector<std::vector<double>> e;
	e = desired_directions(m_states);

	// F^0_a = 1/tau(v^0_a*e-v_a)
	std::vector<std::vector<double>> F(m_disired_direct.size());
	unsigned int i, j;
	for (i = 0; i < m_disired_direct.size(); i++) {
		F[i].push_back((m_speeds[i] * e[i][0] - m_states[i][2]) / m_tau);
		F[i].push_back((m_speeds[i] * e[i][1] - m_states[i][3]) / m_tau);
		//std::cout << F[i][0] << " " << F[i][1] << std::endl;
	}

	// compute f_ab
	f_ab = temp_ped_ped.grad_r_ab(m_states, e, speeds(m_states));
	f_ab = in_sight(f_ab, e);
	
	// compute f_aB
	std::vector<std::vector<std::vector<double>>>f_aB;
	if (m_spaces.size() != 0) {
		ped_space_potential temp_ped_space(m_spaces);
		f_aB = temp_ped_space.grad_r_ab(m_states);
		/*for (i = 0; i < f_aB.size(); i++) {
			std::cout << f_aB[i][0][0] << " " << f_aB[i][0][1] << "\n";
		}*/
	}
	


	// compute social force ---------
	for (i = 0; i < F.size(); i++) {
		for (j = 0; j < F.size(); j++) {
			// social force
			F[i][0] += f_ab[i][j][0];
			F[i][1] += f_ab[i][j][1];
		}
		if (m_spaces.size() != 0 && f_aB.size() != 0) {
			F[i][0] += f_aB[i][0][0];
			F[i][1] += f_aB[i][0][1];
		}

		// desired velocity
		F[i][0] = F[i][0] * m_delta_t + m_states[i][2];
		F[i][1] = F[i][1] * m_delta_t + m_states[i][3];
		//std::cout << F[i][0] << " " << F[i][1] << std::endl;
	}

	// compute vector v
	double temp_g;
	std::vector<std::vector<double>> v(F.size());
	for (i = 0; i < F.size(); i++) {
		temp_g = sqrt(pow(F[i][0], 2) + pow(F[i][1], 2));

		temp_g = std::min(1.0, (m_max_speed_multiplier * m_speeds[i] /temp_g));
		//std::cout << m_speeds[i] << std::endl;
		v[i].push_back(F[i][0] * temp_g);
		v[i].push_back(F[i][1] * temp_g);
		//std::cout << v[i][0] << " " << v[i][1] << std::endl;
	}
	
	// update states
	for (i = 0; i < F.size(); i++) {
		// positions
		m_states[i][0] += v[i][0] * m_delta_t;
		m_states[i][1] += v[i][1] * m_delta_t;
		// velocity
		m_states[i][2] = v[i][0];
		m_states[i][3] = v[i][1];
		//std::cout << F[i][0] << " " << F[i][1] << std::endl;
	}

	/*for (int i = 0; i < m_states.size(); i++){
		for (int j = 0; j < m_states[i].size(); j++)
		{
			std::cout << m_states[i][j] << " ";
		}
		std::cout << std::endl;
	}*/


	return m_states;
}