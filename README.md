### RANSAC Random Sample Consensus (RANSAC)

### Two Implementations 
1. Linear      
2. Circular 

### Libraries
-Boost.Geometry 1.72.0 

### Algorithms 
 #### Inputs 
  1. .CSV file with 2d data points
  2. Number of iterations
  3. Minimum number of inliers
  4. Inlier error threshold 
 #### Outputs
  1. Model parameters 
  2. Number of inliers 
  3. Total error for all inliers
 #### Description 
  1. Create a proposed model (e.g. line, circle) using the minimum number of data points randomly selected from a .CSV file. 
  2. Check every data point & determine if it qualifies as inlier based on the proposed model determined in step 1 & the inlier error  threshold. 
  3. If the number of inliers is >= the minimum number required:
    a. Save the model & its inliers.
    b. Calculate a better model using the minimum number of data points randomly selected from the inliers. 
    c. Calculate the error for each inlier with respect to the better model.  
  4. If the total error for the new model is < the total error for the best model then save as best model. 
  5. Iterate until the number of iterations is reached. 

 #### Error Calculation
 Linear Models -    distance b/w the model of a line and/or a data point/inlier
 Circular Models -  distance b/w the model of a circle and/or a data point/inlier
 
### Use: 
1. User instantiates an object of the linear_ransac or circular_ransac class. 
  a. The constructors can accept:
    1. The path the data
    2. The minimum inliers
    3. The inlier error threshold 
    4. The # of iterations

#### Note: this implementation is not adaptive and dose not guide the user in any way with respect to the selection of appropriate parameters e.g. (the # of iterations, the # of inliers or the inlier error threshold). This functionality may be added at a later time. 
 
  #### Example: 
  
  Create an object of class LinearRansac:
  ```
  LinearRansac lo {ReadCsv ("../data/point_data_linear.csv"), min_inliers, thres, numb_iter};
      // where min_inliers & number_iter are unsigned integers and thres is a double
  ```
  Create a base class pointer:
  ```
  Ransac *ro = &lo;
  ```
  Call the Iterative fit function:
  ```
  r1->IterativeFit()
  ```
  Print out the results to console:
  ```
  cout << lo;
  ```

