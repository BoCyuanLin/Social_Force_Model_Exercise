#include<math.h>
#include<algorithm>

// class: implement the pedestrain with pedestrain
class ped_ped_potential {
public:
	// constructor
	ped_ped_potential(double delta_t):m_delta_t(delta_t) {};
	std::vector<std::vector<std::vector<double>>> r_ab(std::vector<std::vector<double>>);
	std::vector<std::vector<std::vector<double>>> grad_r_ab(std::vector<std::vector<double>>,
		std::vector<std::vector<double>>, std::vector<double>, double);

private:
	// initialize some parameters in paper
	double m_delta_t;
	double m_v0 = 2.1;
	double m_sigma = 0.3;
	//-------------------------------------------

	std::vector<std::vector<double>> value_r_ab(std::vector<std::vector<std::vector<double>>>,
		std::vector<std::vector<double>>, std::vector<double>, double, double);
	

};


std::vector<std::vector<std::vector<double>>> ped_ped_potential::r_ab(std::vector<std::vector<double>> state) {

	std::vector<std::vector<std::vector<double>>> temp_r_ab(state.size(), std::vector<std::vector<double>>(state.size()));
	unsigned int i, j;
	double x, y;

	for (i = 0; i < state.size(); i++) {
		for (j = 0; j < state.size(); j++) {
			x = state[i][0] - state[j][0];
			y = state[i][1] - state[j][1];
			temp_r_ab[i][j].push_back(x);
			temp_r_ab[i][j].push_back(y);
		}
	}

	return temp_r_ab;
}

std::vector<std::vector<double>> ped_ped_potential::value_r_ab(std::vector<std::vector<std::vector<double>>> r, 
	std::vector<std::vector<double>> disired_direction, std::vector<double> speeds, double dx = 0.0, double dy = 0.0) {

	std::vector<std::vector<double>> temp_r_ab(r.size(), std::vector<double>(r.size()));
	unsigned int i, j;
	double temp_norm_f, temp_norm_s, temp_norm_t, temp_norm_s_x, temp_norm_s_y, b; // terms inside the sqrt

	for (i = 0; i < temp_r_ab.size(); i++) {
		for (j = 0; j < temp_r_ab.size(); j++) {

			if (i != j) {
				// compute b in the papper 
				temp_norm_f = sqrt(pow((r[i][j][0] + dx), 2) + pow((r[i][j][1] + dy), 2)); // ||r_ab||
				temp_norm_s_x = ((r[i][j][0] + dx) - speeds[j] * m_delta_t * disired_direction[j][0]);
				temp_norm_s_y = ((r[i][j][1] + dy) - speeds[j] * m_delta_t * disired_direction[j][1]);
				temp_norm_s = sqrt(pow(temp_norm_s_x, 2) + pow(temp_norm_s_y, 2));// ||r_ab-v_b*delta_t*e_b||

				temp_norm_t = pow(speeds[j] * m_delta_t, 2);// (v_b*delta_t)^2

				// compute V^0_ab*e^(-b/sigma) in the papper 
				b = sqrt(pow(temp_norm_f + temp_norm_s, 2) - temp_norm_t) / 2; // semi-minor axis in paper
				temp_r_ab[i][j] = m_v0 * exp(-(b / m_sigma));
			}
			else {
				//b = 0; 
				temp_r_ab[i][j] = 0; // set diagonal 0
			}
			//std::cout << temp_r_ab[i][j] << " ";
		}
		//std::cout << "\n";
	}
	return temp_r_ab;
}

// function: compute the gradient with r_ab using finite difference differentiation
std::vector<std::vector<std::vector<double>>> ped_ped_potential::grad_r_ab(std::vector<std::vector<double>> state,
	std::vector<std::vector<double>> disired_direction, 
	std::vector<double> speeds, double delta = 0.001) {


	
	// comput ra - rb
	std::vector<std::vector<std::vector<double>>> r;
	r = r_ab(state);

	// compute the r_ab
	std::vector < std::vector<double>> v, dvdx, dvdy;
	v = value_r_ab(r, disired_direction, speeds);

	// compute the gradient
	dvdx = value_r_ab(r, disired_direction, speeds, delta, 0.0);
	dvdy = value_r_ab(r, disired_direction, speeds, 0.0, delta);

	unsigned int i, j;

	for (i = 0; i < v.size(); i++){
		for (j = 0; j < v[i].size(); j++){
			dvdx[i][j] = (dvdx[i][j] - v[i][j]) / delta;
			dvdy[i][j] = (dvdy[i][j] - v[i][j]) / delta;
		}
	}

	std::vector<std::vector<std::vector<double>>> dvdx_dvdy(v.size(), std::vector<std::vector<double>>(v.size()));
	
	for (i = 0; i < v.size(); i++) {
		for (j = 0; j < v[i].size(); j++) {
			dvdx_dvdy[i][j].push_back(-dvdx[i][j]);
			dvdx_dvdy[i][j].push_back(-dvdy[i][j]);
			//std::cout << dvdx_dvdy[i][j][0] << " " << dvdx_dvdy[i][j][1] << "\n";
		}
	}
	// release the memory
	/*r.clear();
	r.shrink_to_fit();*/
	

	return dvdx_dvdy;
}

// ******************************************************************************************


// class: implement the pedestrain with space
class ped_space_potential {
public:
	// constructor
	ped_space_potential(std::vector<std::vector<double>> spaces) :m_spaces(spaces) {};
	std::vector<std::vector<double>> r_ab(std::vector<std::vector<double>>);
	std::vector<std::vector<std::vector<double>>> grad_r_ab(std::vector<std::vector<double>>, double);

private:
	// initialize some parameters in paper
	std::vector<std::vector<double>> m_spaces;
	double m_u0 = 10;
	double m_R = 0.2;
	//-------------------------------------------

	std::vector<std::vector<double>> value_r_ab(std::vector<std::vector<double>>, double, double);

};

std::vector<std::vector<double>> ped_space_potential::r_ab(std::vector<std::vector<double>> state) {

	std::vector<std::vector<double>> temp_r_ab(state.size());
	//std::vector<unsigned int> closest_index(state.size());
	std::vector<std::vector<double>>temp_norms(state.size(), std::vector<double>(m_spaces.size()));
	unsigned int i, j, temp_argmin;
	double x, y;
	//std::cout << temp_norms.size() << temp_norms[0].size() <<  "\n";
	//temp_norm = sqrt(pow(state[0][0]-m_spaces[0][0], 2) + pow(state[0][1] - m_spaces[0][1], 2));

	// compute norms of points between spaces
	for (i = 0; i < temp_norms.size(); i++) {
		for (j = 0; j < temp_norms[i].size(); j++) {
			temp_norms[i][j] = sqrt(pow(state[i][0] - m_spaces[j][0], 2) + pow(state[i][1] - m_spaces[j][1], 2));
		}
		temp_argmin = std::distance(temp_norms[i].begin(), std::min_element(temp_norms[i].begin(), temp_norms[i].end()));
		//std::cout << temp_argmin << std::endl;
		temp_r_ab[i].push_back(state[i][0] - m_spaces[temp_argmin][0]);
		temp_r_ab[i].push_back(state[i][1] - m_spaces[temp_argmin][1]);
		//std::cout << temp_r_ab[i][0] << " " << temp_r_ab[i][1] << "\n";
	}

	return temp_r_ab;
}

std::vector<std::vector<double>> ped_space_potential::value_r_ab(std::vector<std::vector<double>> r, double dx = 0.0, double dy = 0.0) {

	std::vector<std::vector<double>> temp_r_ab(r.size());
	unsigned int i, j;
	double temp_norm; // terms inside the sqrt

	for (i = 0; i < temp_r_ab.size(); i++) {
		

		// compute U^0_ab*e^(-||r_ab||/R) in the papper 
		temp_norm = sqrt(pow(r[i][0]+dx, 2) + pow(r[i][1]+dy, 2)); // semi-minor axis in paper
		temp_r_ab[i].push_back(m_u0 * exp(-(temp_norm / m_R)));
			
		//std::cout << temp_r_ab[i][0] << "\n";
	}
	return temp_r_ab;
}

// function: compute the gradient with r_ab using finite difference differentiation
std::vector<std::vector<std::vector<double>>> ped_space_potential::grad_r_ab(std::vector<std::vector<double>> state, double delta = 0.001) {

	// comput ra - rb
	std::vector<std::vector<double>> r;
	r = r_ab(state);


	// compute the r_ab
	std::vector < std::vector<double>> v, dvdx, dvdy;
	v = value_r_ab(r);

	// compute the gradient
	dvdx = value_r_ab(r, delta, 0.0);
	dvdy = value_r_ab(r, 0.0, delta);

	unsigned int i, j;

	for (i = 0; i < v.size(); i++) {
		for (j = 0; j < v[i].size(); j++) {
			dvdx[i][j] = (dvdx[i][j] - v[i][j]) / delta;
			dvdy[i][j] = (dvdy[i][j] - v[i][j]) / delta;
		}
		//std::cout << dvdx[i][0] << " " << dvdy[i][0] << "\n";
	}

	std::vector<std::vector<std::vector<double>>> dvdx_dvdy(v.size(), std::vector<std::vector<double>>(v.size()));

	for (i = 0; i < v.size(); i++) {
		for (j = 0; j < v[i].size(); j++) {
			dvdx_dvdy[i][j].push_back(-dvdx[i][j]);
			dvdx_dvdy[i][j].push_back(-dvdy[i][j]);
			//std::cout << dvdx_dvdy[i][j][0] << " " << dvdx_dvdy[i][j][1] << "\n";
		}
	}
	

	return dvdx_dvdy;
}


// ******************************************************************************************