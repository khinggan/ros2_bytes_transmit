import pickle

# Example data
data = {'name': 'khinggan', 'age': 28, 'city': 'Tokyo'}

# Writing data to a pickle file
with open('example.pkl', 'wb') as file:
    pickle.dump(data, file)
