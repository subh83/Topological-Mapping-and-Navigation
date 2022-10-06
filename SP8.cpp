/* c++ -std=c++11 -g -I. -Idosl SP8.cpp -lopencv_core -lopencv_highgui -lopencv_imgproc -lpthread -DARMA_DONT_USE_WRAPPER -lopenblas -llapack */
// c++ -std=c++11 -g -I. -Idosl SP8.cpp -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lpthread -DARMA_DONT_USE_WRAPPER -lopenblas -llapack

// for j in `seq 1 1 10`; do for i in `seq 0 50 1000`; do ./a.out $i; done; done

#include "1.Libraries.h"
#include "2.User_Defined_Classes.h"
#include "3.User_Defined_Functions.h"
#include "4.Exploration_Checks.h"
#include "5.Robot_Path_Planning.h"
#include "6.Landmark_Placement.h"

bool live_plot=true;

int main(int argc, char* argv[]) {

	factor = 1;//atof(argv[1]);
	//num_ISW = 20; //SB //2;//atof(argv[2]);
	//num_RW = 1;//atof(argv[3]);
	
	// SB
	/*if ( argc > 1 ) {
        max_initial_rw = atoi( argv[1] );
    }
    if ( argc > 2 ) {
        first_percent_complete = atof( argv[2] );
    }*/
    //cout << "max_initial_rw: " << max_initial_rw << endl;
    
	//cout << factor << ", " << num_ISW << ", " << num_RW << endl;
	
	// SB:
	if ( argc > 1 ) {
        rbt = atoi( argv[1] );
    }
    HIW_finished = vector<bool>(rbt, false);
    RWISW_finished = vector<bool>(rbt, false);
	
	cout << rbt << " robots in " << image_name << " with landmarks in " << text_name << endl;

	vector<thread> threads;

	for (int p = 0; p < rbt; ++p) {
		A:
		int x = rand()%ref_image.cols;
		int y = rand()%ref_image.rows;
		if (ifl_image.at<Vec3b> (y, x) == Vec3b(255, 255, 255))
			observations.push_back(dpoint (x, y, (((double)rand())/((double)RAND_MAX))*2*PI-PI,
								sensor_radius, sensor_angle, p));
		else
			goto A;
	}

//	observations.push_back(dpoint (300, 200, (((double)rand())/((double)RAND_MAX))*2*PI-PI,
//		sensor_radius, sensor_angle, 0));

	read_landmarks (text_name, landmarks);
	for (int a = 0; a < landmarks.size(); ++a) {
		obs_list[0].insert(landmarks[a].id);
	}

	for (int r = 0; r < rbt; ++r)
		threads.push_back (thread (move_robot, observations[r].rbt_id));

	sleep(1);
//	file.open("Results/file.txt");
//	file_ratio.open("Results/file_ratio.txt");
	iterations_ct = 0;

	Mat IMG1(ref_image.rows, ref_image.cols, CV_8UC3, Scalar (255, 255, 255));
	Mat IMG2(ref_image.rows, ref_image.cols, CV_8UC3, Scalar (255, 255, 255));
//	Mat IMG1 = imread(image_name, CV_LOAD_IMAGE_COLOR);
//	Mat IMG2 = imread(image_name, CV_LOAD_IMAGE_COLOR);

	int previous_expl_count(0);
	int previous_sc_count(0);
	auto start = std::chrono::high_resolution_clock::now();
    
    int landmarks_obs_ct = 0;
    
	while (!Thread_finished_check(RWISW_finished)) {

		img_mtx.lock();
		sc_image = 0.95*sc_image + 0.05*PthHmlgy_image;
		PthHmlgy_image = imread(nice_image_name);
		img_mtx.unlock();
		
		// SB:
		/*for (int i = 0; i != landmarks.size(); ++i)
		    landmarks_obs_ct += landmarks[i].obs_count;*/

		cout<<"sc.C[2].size(): "<<sc.C[2].size()<<", "<<C2_size<<", expl_count: "<<expl_count
		    //<<", total_landmark_obs_count: "<<landmarks_obs_ct // SB
		    <<", rt: "<<rt // SB
			<<", percentage: "<<(double(C2_size)/sc_number)*100<<"%"<<"\r"<<flush;

		if (iterations_ct%1000 == 0) {

//			file<<expl_count<<"\t"<<(double(sc.C[2].size())/sc_number)*100<<endl;

//			sc.draw_simplicial_complex (IMG1, landmarks);

			rt = double(C2_size-previous_sc_count)/double(expl_count-previous_expl_count);
			//cout << "\nit: " << it << "\trt: " << rt << endl;// SB
				//"\t" << white_pixel_count(IMG1) <<endl;

			previous_expl_count = expl_count;
			previous_sc_count = C2_size;
		}
		
		// SB:
		/*if ((double(C2_size)/sc_number)*100 > 50) {
		    auto stop = std::chrono::high_resolution_clock::now();
		    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
		    cout<<"\n\n"
		    <<"max_initial_rw: " << max_initial_rw << ", it_at_end_of_initial_RW: " << it_at_end_of_initial_RW
		    << "\ngamma_percentage: "<<gamma_percentage << ", expl_count_at_initial_RW_end: " << expl_count_at_initial_RW_end
		    <<"\nit: "<< iterations_ct << ", time (s): "<< ((double)(duration.count())) / 1000000.0
		    <<"\nsc.C[2].size(): "<<sc.C[2].size()<<", "<<C2_size<<", expl_count: "<<expl_count
		    <<", total_landmark_obs_count: "<<landmarks_obs_ct // SB
			<<", percentage: "<<(double(C2_size)/sc_number)*100<<"%"<<"\r"<<flush;
			cout << "\nSB: 50% exploration complete. Exitting!\n";
			
            std::ofstream outfile;
            outfile.open("gamma.txt", std::ios_base::app); // append instead of overwrite
            outfile << max_initial_rw <<", " << iterations_ct << ", " << it_at_end_of_initial_RW << "\n";
			
			return(0);
		}*/

		iterations_ct++;
        
        if (live_plot) {
		    imshow ("img0", sc_image);
		    imshow ("simplex_image", simplex_image);
		    //if (iterations_ct%1000 == 0) imwrite("LCCA_result.jpg", simplex_image);
		}
		waitKey(animation_wait);

	}

	RWISW_finished.assign(rbt, false);

	cout << endl;
	cout << "RW&ISW sc/obs, " << iterations_ct << ": " <<
		double(C2_size-previous_sc_count)/double(expl_count-previous_expl_count)
		<< endl;
	
	// SB:
    /*auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    for (int i = 0; i != landmarks.size(); ++i)
	    landmarks_obs_ct += landmarks[i].obs_count;
	    
    std::ofstream outfile;
    outfile.open("rbt2.txt", std::ios_base::app); // append instead of overwrite
    outfile << rbt << ", " // input param
            << (double(C2_size)/sc_number)*100 << ", " // % of complex completed at this entry
            << iterations_ct << ", " // number of iterations at this entry
            << landmarks_obs_ct << ", "
            << expl_count << ", "
            << ((double)(duration.count())) / 1000000.0
            << "\n";
	return(0);*/
	
	// SB:
	/*auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    for (int i = 0; i != landmarks.size(); ++i)
	    landmarks_obs_ct += landmarks[i].obs_count;
	    
    std::ofstream outfile;
    outfile.open("gamma_full_new3b.txt", std::ios_base::app); // append instead of overwrite
    outfile << max_initial_rw << ", " // input param
            << (double(C2_size)/sc_number)*100 << ", " // % of complex completed at this entry
            << iterations_ct << ", " // number of iterations at this entry
            << it_at_end_of_initial_RW << ", " // 0, or the iteration at the end of RW
            << landmarks_obs_ct << ", "
            << ((double)(duration.count())) / 1000000.0
            << "\n";
	return(0);*/
	
	sc.draw_simplicial_complex (IMG1, landmarks);
	if (live_plot) {
	    imshow("IMG1", IMG1);
	    //if (iterations_ct%1000 == 0) imwrite("LCCA_result.jpg", IMG1);
    }
	waitKey (2e3);

//	IMG2 = IMG1.clone();
//	cout << "white pixel left: " << white_pixel_count(IMG2) << endl;

//	file<<endl;
//	file_ratio<<endl;

	while ((double(C2_size)/sc_number) < third_percent_complete) { //white_pixel_count(IMG2) > 100

		homology_step++;
		sc.Homology ();

//		myfile.open("Results/res.txt", ios_base::app);
//		myfile<<factor<<"\t"<<num_ISW<<"\t"<<num_RW<<"\t"<<expl_count<<endl;

		while (!Thread_finished_check(HIW_finished)) {

			for (int r = 0; r < rbt; ++r) {
				if (!HIW_finished[r] && !observations[r].Assigned) {
					threads.push_back (thread (
						updated_Homology_Informed_Walk, observations[r].rbt_id));
				}
			}

			cout<<"sc.C[2].size(): "<<sc.C[2].size()<<", "<<C2_size<<", expl_count: "
				<<expl_count<<", percentage: "<<(double(C2_size)/sc_number)*100
				<<"%"<<"\r"<<flush;

			if (iterations_ct%1000 == 0) {

/*				file_ratio<<it<<"\t"<<
					double(sc.C[2].size()-previous_sc_count)/
					double(expl_count-previous_expl_count)*100
					<<endl;
*/
				previous_expl_count = expl_count;
				previous_sc_count = C2_size;
			}

			iterations_ct++;

			img_mtx.lock();
			sc_image = 0.95*sc_image + 0.05*PthHmlgy_image;
			PthHmlgy_image = imread(nice_image_name);
			img_mtx.unlock();
			imshow ("img0", sc_image);
			imshow ("simplex_image", simplex_image);
			//if (iterations_ct%1000 == 0) imwrite("LCCA_result.jpg", simplex_image);
			waitKey (animation_wait);

		}

		HIW_finished.assign(rbt, false);

		for (int r = 0; r < rbt; ++r)
			observations[r].Assigned = false;

		cout << endl;
		cout << "HIW sc/obs, " << iterations_ct << ": " <<
			double(C2_size-previous_sc_count)/double(expl_count-previous_expl_count) << endl; 
				//"\t" << white_pixel_count(IMG2) << endl;

		previous_expl_count = expl_count;
		previous_sc_count = C2_size;

		sc.draw_simplicial_complex (IMG2, landmarks);
		imshow("IMG2", IMG2);
		//if (iterations_ct%1000 == 0) imwrite("LCCA_result.jpg", IMG2);
		waitKey (2e3);

	}
    
    /*// SB
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    for (int i = 0; i != landmarks.size(); ++i)
	    landmarks_obs_ct += landmarks[i].obs_count;
	    
    std::ofstream outfile;
    outfile.open("rbt.txt", std::ios_base::app); // append instead of overwrite
    outfile << rbt << ", " // input param
            << (double(C2_size)/sc_number)*100 << ", " // % of complex completed at this entry
            << iterations_ct << ", " // number of iterations at this entry
            << landmarks_obs_ct << ", "
            << expl_count << ", "
            << ((double)(duration.count())) / 1000000.0
            << "\n";
	return(0);*/
    
    
//	cout << "white pixel left: " << white_pixel_count(IMG2) << endl;
	waitKey (0);

return 0; }
