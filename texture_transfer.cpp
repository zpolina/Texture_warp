
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <array>
#include <math.h>
#include <sys/types.h>
#include <dirent.h>
#include <string>
using namespace std;

typedef pair<int, int> key;
typedef map<key, array<pair<int,int> ,5>> dict;

void get_uv_points(cv::Mat I,
                   cv::Mat U,
                   cv::Mat V,
                   int partInd,
                   vector<array<int, 4> >& points){
  //cout << "M = "<< endl << " "  << IUV << endl << endl;
  for(int i = 0; i < I.rows; i++)
    {
      uchar* ii = I.ptr(i);
      uchar* uu = U.ptr(i);
      uchar* vv = V.ptr(i);
      for(int j = 0; j < I.cols; j++){
        if (static_cast<int>(*ii) == partInd){
            //cout<<static_cast<int>(*uu)<<endl;
            int first_val = static_cast<int>(*uu);
            int second_val = static_cast<int>(*vv);
            array<int, 4> new_arr = {first_val, second_val, i, j};
            points.push_back(new_arr);
        }
        ii++;
        uu++;
        vv++;
      }
    }
}

double distanceCalculate(double x1,  double x2, double y1, double y2)
{
	double x = x1 - x2;
	double y = y1 - y2;
	double dist = x*x + y*y;
	return dist;
}

void get_xy_points(cv::Mat I,
                   int partInd,
                   vector<std::pair<int, int> >& points){
  for(int i = 0; i < I.rows; i++)
    {
      uchar* ii = I.ptr(i);
      for(int j = 0; j < I.cols; j++){
        if (static_cast<int>(*ii) == partInd){
            points.push_back(std::make_pair(i, j));
        }
        ii++;
      }
    }
}

pair<int, int> centroid(std::array<pair<int, int>,5> v, int len){
  int val1 = 0, val2 =0;
  for(auto & el: v){
    val1+= el.first;
    val2+= el.second;
  }
  return make_pair(static_cast<int>(val1/len), static_cast<int>(val2/len));
}

pair<double, int> find_min_distance(array<array<double, 4>, 5> & distances){
  double minimum = 10000000000;
  double maximum = 0;
  int idx = 0;
  int i =0;
  for(auto& el: distances){
    if(el[0] < minimum){
      minimum = el[0];
    }
    if(el[0] > maximum){
      maximum = el[0];
      idx = i;
    }
    i++;
  }
  return make_pair(minimum, idx);
}


bool compareCoordinates(pair<int,int> p1, pair<int, int> p2)
{
    return (p1.first > p2.first);
}
template <class T>
bool compareDistances(array<T,4> d1, array<T, 4> d2)
{
    return (d1[0] < d2[0]);
}



int main( int argc, char** argv ) {

  // Check the number of parameters
   if (argc < 4) {
       // Tell the user how to run the program
       std::cout << "Please enter new image size, i.e two number hight and width and enter the name of the file of the folder" << std::endl;
       return 1;
  }
  string name = "2";
  string name_target = "2";
  const char* directoryName = argv[1];
  int height = atoi(argv[2]);
  int width = atoi(argv[3]);
  int datset_size = atoi(argv[4]);
  //parsing the directory
  vector<string> directories_num;
  DIR *directory = opendir( directoryName );

  if( directory == NULL )
      {
      perror( directoryName );
      exit( -2 );
      }
  struct dirent *entry;
  while( NULL != ( entry = readdir(directory) ) )
  {
    if( entry->d_name[0] != '.'  )
      {
        directories_num.push_back(entry->d_name);
      }
  }


  cv::Mat orig_im_1, orig_IUV_1, IUV_1;
  cv::Mat orig_im, orig_IUV, IUV;

  std::vector<cv::Mat> channels_IUV;
  std::vector<cv::Mat> channels_orig_IUV;
  std::vector<cv::Mat> channels_orig_im;
  cv::Mat R_im = cv::Mat(width,height ,CV_8UC1);
  cv::Mat G_im = cv::Mat(width,height ,CV_8UC1);
  cv::Mat B_im = cv::Mat(width,height ,CV_8UC1);
  vector<std::array<int, 4> > list_new;
  vector<std::pair<int,int> > pixels_pairs_old;
  std::array<pair<int, int>,5> prev_points;
  std::array<pair<int, int>,5> empty_arr;
  std::array<double, 4> empty_arr_2;
  std::array<pair<int, int>,5> points;
  dict prev_points_dict;
  pair<double, int> min_idx;
  //vector <string> originals = {"/p-4x.jpg", "/p-4x.jpg", "/1-4x.jpg", "/1-4x.jpg", "/3-4x.jpg", "/3-4x.jpg"};
  //vector <string> original_iuvs = {"/p-4x_IUV.png", "/p-4x_IUV.png", "/1-4x_IUV.png", "/1-4x_IUV.png", "/3-4x_IUV.png", "/3-4x_IUV.png"};
  //vector <string> target_iuvs = { "/1-4x_IUV.png", "/3-4x_IUV.png", "/p-4x_IUV.png",  "/3-4x_IUV.png", "/p-4x_IUV.png", "/1-4x_IUV.png"};
  //vector <string> final_names = {"p-1.jpg", "p-3.jpg", "1-p.jpg", "1-3.jpg", "3-p.jpg", "3-1.jpg"};
  //for(int i =0; i<5; i++){
  //TODO: change the maximum index
    for (int names_idx =1; names_idx<datset_size+1; names_idx++){
      orig_im_1  = cv::imread(static_cast<string>(directoryName) + "source/frame"  + std::to_string(names_idx) + ".jpg", CV_LOAD_IMAGE_COLOR);
      orig_IUV_1 = cv::imread(static_cast<string>(directoryName) + "dp_source/frame"+ std::to_string(names_idx)+ ".png", CV_LOAD_IMAGE_COLOR);
      IUV_1 = cv::imread(static_cast<string>(directoryName) + "dp_target/frame"+ std::to_string(names_idx)+ ".png", CV_LOAD_IMAGE_COLOR);
      ///cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
      //cv::imshow( "Display window", rrrr);
      //
      //cv::waitKey(0);
      cout<<static_cast<string>(directoryName) + "dp_source/frame"+ std::to_string(names_idx)+ ".png"<<endl;
      if(! (orig_im_1.data && orig_IUV_1.data && IUV_1.data)) {
          std::cout <<  "Could not open or find the image" << std::endl ;
          return -1;
      }


      cv::Size size(height, width);
      cv::resize(orig_im_1, orig_im, size);
      cv::resize(orig_IUV_1, orig_IUV, size);
      cv::resize(IUV_1, IUV, size);


      channels_IUV.clear();
      cv::split(IUV, channels_IUV);
      cv::Mat I = channels_IUV[0];
      cv::Mat U = channels_IUV[1];
      cv::Mat V = channels_IUV[2];

      channels_orig_IUV.clear();
      cv::split(orig_IUV, channels_orig_IUV);
      cv::Mat I_orig = channels_orig_IUV[0];
      cv::Mat U_orig = channels_orig_IUV[1];
      cv::Mat V_orig = channels_orig_IUV[2];

      channels_orig_im.clear();
      cv::split(orig_im,channels_orig_im);
      cv::Mat R = channels_orig_im[0];
      cv::Mat G = channels_orig_im[1];
      cv::Mat B = channels_orig_im[2];

      R_im.setTo(cv::Scalar(255));
      G_im.setTo(cv::Scalar(255));
      B_im.setTo(cv::Scalar(255));


      empty_arr.fill(make_pair(0, 0));
      empty_arr_2.fill(0);
      for(size_t partInd = 1; partInd < 25; ++partInd){
        cout<<"Part ID"<< partInd<<endl;
        list_new.clear();
        pixels_pairs_old.clear();
        prev_points.fill(make_pair(0, 0));
        empty_arr.fill(make_pair(0, 0));
        points.fill(make_pair(0, 0));
        get_uv_points(I, U, V, partInd, list_new);
        get_xy_points(I_orig, partInd, pixels_pairs_old);
        sort(list_new.begin(), list_new.end(), compareDistances<int>);
        sort(pixels_pairs_old.begin(), pixels_pairs_old.end());
        prev_points_dict.clear();
        for(vector<array<int, 4> > ::size_type i = 0; i != list_new.size(); i++) {
            int u_cur = list_new[i][0];
            int v_cur = list_new[i][1];
            bool exist_prev_point = false;

            for(int x = 0; x > -4; x--){
              for(int y = 0; y< 4; y++){
                pair<int, int> p = make_pair<int, int>(u_cur + x, v_cur +y);
                if (prev_points_dict.find(p) != prev_points_dict.end()){
                  exist_prev_point = true;
                  prev_points = prev_points_dict[p];
                  break;
                }
                p = make_pair<int, int>(u_cur + x, v_cur - y);
                if (prev_points_dict.find(p) != prev_points_dict.end()){
                  exist_prev_point = true;
                  prev_points = prev_points_dict[p];
                  break;
                }
              }
              if(exist_prev_point)
                break;
            }
            array<array<double, 4>, 5> distances;
            for(int d = 0; d < 5; ++d){
              distances[d][0] = 100000000;
              for(int f =1; f < 4; ++f){
                  distances[d][f] = 0;
              }
            }
            if (exist_prev_point && prev_points != empty_arr){
              points.fill(make_pair(0, 0));
              sort(prev_points.begin(), prev_points.end(), compareCoordinates);
              //for(auto& el: prev_points){
                //cout<<el.first<< "   "<<el.second<<"        ";
              //}
              //cout<<endl;
              auto it = std::find(prev_points.begin(), prev_points.end(), make_pair(0,0));
              int leng = 5;
              if (it != prev_points.end())
              {
                leng = std::distance(prev_points.begin(), it);
              }
              pair<int,int> c = centroid(prev_points, leng);
              //TODO make the size of radious program variable
              int radius_r = min(c.first + 10, I.cols) - c.first;
              int radius_u = min(c.second + 10, I.rows) - c.second;
              int radius_l = max(c.first - 10, 0) - c.first;
              int radius_d = max(c.second - 10, 0) - c.second;
              for(int x = radius_l; x< radius_r; x++){
                for(int y = radius_d; y < radius_u; y++){
                  if(static_cast<int>(I_orig.at<uchar>(c.first+ x, c.second + y)) == partInd){
                    int u = static_cast<int>(U_orig.at<uchar>(c.first+ x, c.second + y));
                    int v = static_cast<int>(V_orig.at<uchar>(c.first+ x, c.second + y));
                    double d = distanceCalculate(u, u_cur, v, v_cur );
                    min_idx = find_min_distance(distances);
                    //cout<<"minimum index"<< min_idx.first<< "   "<<min_idx.second<<endl;
                    if(d < min_idx.first){
                      distances[min_idx.second][0] = d;
                      distances[min_idx.second][1] =
                      static_cast<int>(R.at<uchar>(c.first+ x, c.second + y));
                      distances[min_idx.second][2] =
                      static_cast<int>(G.at<uchar>(c.first+ x, c.second + y));
                      distances[min_idx.second][3] =
                      static_cast<int>(B.at<uchar>(c.first+ x, c.second + y));
                      points[min_idx.second] = make_pair(c.first+ x, c.second + y);
                    }
                  }
                }
              }
              prev_points_dict.insert(make_pair(make_pair(list_new[i][0],list_new[i][1]), points));
            }
            else{
              points.fill(make_pair(0, 0));
              for(auto& pixel_old: pixels_pairs_old){
                int u =  static_cast<int>(U_orig.at<uchar>(pixel_old.first, pixel_old.second));
                int v =  static_cast<int>(V_orig.at<uchar>(pixel_old.first, pixel_old.second));
                double d = distanceCalculate(u, u_cur, v, v_cur);
                min_idx = find_min_distance(distances);
                if(d < min_idx.first){
                  distances[min_idx.second][0] = d;
                  distances[min_idx.second][1] = static_cast<int>(R.at<uchar>(pixel_old.first, pixel_old.second));
                  distances[min_idx.second][2] = static_cast<int>(G.at<uchar>(pixel_old.first, pixel_old.second));
                  distances[min_idx.second][3] = static_cast<int>(B.at<uchar>(pixel_old.first, pixel_old.second));
                  points[min_idx.second] = make_pair(pixel_old.first, pixel_old.second);
                  //cout<<d<<endl;
                }
              }
              ///////////////////////////
              sort(points.begin(), points.end(), compareCoordinates);
              prev_points_dict.insert(make_pair(make_pair(list_new[i][0],list_new[i][1]), points));
              /*cout<<"Points from slow process"<<endl;
              for(auto& el: points){
                cout<<el.first<< "   "<<el.second<<"        ";
              }
              cout<<endl;*/
            }
            sort(distances.begin(), distances.end(), compareDistances<double>);
            //cout<<distances[0][0]<< "   "<<distances[0][1]<<"  "<<distances[0][2]<<"  "<<distances[0][3]<<"   ";
            //cout<<endl;
            double extR = 0.0, extG = 0.0, extB = 0.0, extW = 0.0;

            auto it = std::find(distances.begin(), distances.end(), empty_arr_2);
            int max_ind = 5;
            if (it != distances.end())
            {
              max_ind = std::distance(distances.begin(), it);
            }
            //cout<<max_ind<<endl;
            double r_current_points= 0.0, g_current_points= 0.0, b_current_points= 0.0;
            for(int k = 0; k < max_ind; k++){
              r_current_points= 0.0; g_current_points= 0.0; b_current_points= 0.0;
              double w = 1 - distances[k][0] / distances[max_ind-1][0];
              //cout<<w<<endl;
              extW += w;
              extR+=w*distances[k][1];
              extG+=w*distances[k][2];
              extB+=w*distances[k][3];
            }
            if (extW > 0){
              r_current_points= extR/extW;
              g_current_points= extG/extW;
              b_current_points= extB/extW;
            }
            //cout<<r_current_points<< "   "<< g_current_points<< "  "<< b_current_points;
            R_im.at<uchar>(list_new[i][2], list_new[i][3]) = static_cast<uchar>(r_current_points);
            G_im.at<uchar>(list_new[i][2], list_new[i][3]) = static_cast<uchar>(g_current_points);
            B_im.at<uchar>(list_new[i][2], list_new[i][3]) = static_cast<uchar>(b_current_points);

          }
        }
      cv::Mat result_matrix;
      vector<cv::Mat> channels;
      channels.push_back(R_im);
      channels.push_back(G_im);
      channels.push_back(B_im);
      merge(channels, result_matrix);

      //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
      //cv::imshow( "Display window", result_matrix);
      //cv::waitKey(0);
      imwrite(static_cast<string>(directoryName) + "texture/frame"  + std::to_string(names_idx)+ ".jpg", result_matrix);
    }
  return 0;
}
