import scipy.io as sio
import numpy as np
from sklearn.linear_model import LinearRegression
import scipy.io as sio

def read_data_from_mat(file_name):
    # Load the .mat file
    mat_data = sio.loadmat(file_name)
    
    # Access the data
    data1 = mat_data['pos']
    data2 = mat_data['tor']
    data3 = mat_data['gcomp']
    
    # Convert data to more user-friendly formats (optional)
    data1 = data1.flatten()  # Flatten to a 1D array
    data2 = data2.flatten()
    data3 = data3.flatten()

    return data1, data2, data3

# Function to perform linear regression to compute k
def compute_k(pos, tor, gcomp):
    # We want to fit the equation: gcomp = tor - k * pos
    # Rearrange to match: gcomp = -k * pos + tor, where tor is the intercept and -k is the coefficient of pos
    
    # Prepare data for regression (X is pos, and y is gcomp - tor)
    X = pos.reshape(-1, 1)  # Make pos a 2D array
    y = gcomp - tor  # This is the adjusted target variable

    # Create a LinearRegression model
    model = LinearRegression()

    # Fit the model
    model.fit(X, y)

    # Get the model parameters
    intercept = model.intercept_  # This should represent tor
    k = -model.coef_[0]  # The coefficient for pos is -k, so we negate it

    print(f"Intercept (tor): {intercept}")
    print(f"Coefficient (k): {k}")

    return intercept, k

def compute_k_normal_equation(pos, tor, gcomp):
    # Prepare the data for the linear regression (X is pos, and y is gcomp - tor)
    X = pos.reshape(-1, 1)  # Reshape pos to a 2D array (n_samples, 1)
    y = gcomp - tor  # Adjusted target variable (gcomp - tor)

    # Normal Equation: theta = (X.T * X)^(-1) * X.T * y
    # This solves for k, where the equation is gcomp - tor = -k * pos
    X_transpose = X.T
    theta = np.linalg.inv(X_transpose.dot(X)).dot(X_transpose).dot(y)

    # The value of k is the negative of the coefficient of pos
    k = -theta[0]  # Since the coefficient of pos is -k

    print(f"Coefficient (k): {k}")

    return k

def compute_quadratic_coefficients(pos, tor, gcomp):
    # Prepare the data for quadratic regression
    X = np.c_[pos, pos**2]  # Add a column for intercept, pos, and pos^2
    y = gcomp - tor  # Adjusted target variable (gcomp - tor)

    # Normal Equation: theta = (X.T * X)^(-1) * X.T * y
    X_transpose = X.T
    theta = np.linalg.inv(X_transpose.dot(X)).dot(X_transpose).dot(y)

    a = -theta[0]  # This is the coefficient for pos
    b = -theta[1]  # This is the coefficient for pos^2

    print(f"Coefficient (a for pos): {a}")
    print(f"Coefficient (b for pos^2): {b}")

    return a, b

pos, tor, gcomp = read_data_from_mat("estimate_passive.mat")
print("pos :", pos)
print("tor :", tor)
print("gcomp :", gcomp)

k_a, k_b = compute_quadratic_coefficients(pos, tor, gcomp)
