import pandas as pd
import numpy as np
import plotly.express as px
import plotly.graph_objs as go

# List of case names
case_names = [
    'gc_12_47_1', 'r1_12_47_1', 'r1_12_47_2', 'r1_12_47_3', 'r1_12_47_4',
    'r1_25_00_1', 'r2_12_47_1', 'r2_12_47_3', 'r2_25_00_1',
    'r2_35_00_1', 'r3_12_47_1', 'r3_12_47_2', 'r3_12_47_3',
    'r4_12_47_1', 'r4_12_47_2', 'r5_12_47_3', 'r4_25_00_1',
    'r5_12_47_1', 'r5_12_47_2', 'r5_12_47_4',
    'r5_12_47_5', 'r5_25_00_1', 'r5_35_00_1',
    'network_model', 'network_model_case1', 'network_model_case1_342', 'network_model_case2', 'ieee_four_bus', 'regulator_center_tap_xfmr', 'ieee_four_bus_with_triplex'
]

def load_and_prepare_data(case_index):
    case_name = case_names[case_index]
    voltage_profile_path = f'Combined-txd/test/data/three_phase/{case_name}/result.csv'
    result_path = 'voltage_profile.csv'

    # Load data
    voltage_profile = pd.read_csv(voltage_profile_path, skiprows=2)
    result = pd.read_csv(result_path, skiprows=1)

    # Set column names
    columns = ['node_name', 'voltA_real', 'voltA_imag',
               'voltB_real', 'voltB_imag', 'voltC_real', 'voltC_imag']
    voltage_profile.columns = columns
    result.columns = columns

    # Calculate voltage magnitudes
    for df in [voltage_profile, result]:
        df['voltA_mag'] = np.sqrt(df['voltA_real']**2 + df['voltA_imag']**2)
        df['voltB_mag'] = np.sqrt(df['voltB_real']**2 + df['voltB_imag']**2)
        df['voltC_mag'] = np.sqrt(df['voltC_real']**2 + df['voltC_imag']**2)

    # Merge data on voltage magnitudes
    comparison_df = pd.merge(voltage_profile[['node_name', 'voltA_mag', 'voltB_mag', 'voltC_mag']],
                             result[['node_name', 'voltA_mag', 'voltB_mag', 'voltC_mag']],
                             on='node_name', suffixes=('_vp', '_res'))

    # Calculate differences and percentage differences
    differences = pd.DataFrame()
    percentage_differences = pd.DataFrame()
    differences['node_name'] = comparison_df['node_name']
    percentage_differences['node_name'] = comparison_df['node_name']

    tolerance = 0.1  # Define a tolerance for values close to zero
    for component in ['voltA_mag', 'voltB_mag', 'voltC_mag']:
        differences[f'delta_{component}'] = comparison_df[f'{component}_vp'] - comparison_df[f'{component}_res']
        base_values = comparison_df[f'{component}_vp'].abs()
        percent_delta = 100 * differences[f'delta_{component}'].abs() / base_values
        percent_delta[base_values < tolerance] = 0  # Set percentage difference to zero if base value is close to zero
        percentage_differences[f'percent_delta_{component}'] = percent_delta

    # Export differences to CSV
    differences.to_csv('comparison_differences.csv', index=False)
    percentage_differences.to_csv('comparison_percentage_differences.csv', index=False)

    return differences, percentage_differences

def main():
    # Display case names with indices
    for index, name in enumerate(case_names):
        print(f"{index}: {name}")

    # Get user input for case selection
    case_index = int(input("Enter the number for the case you want to analyze: "))
    differences, percentage_differences = load_and_prepare_data(case_index)

    # Display the calculated differences
    print("Differences:")
    print(differences)
    print("\nPercentage Differences:")
    print(percentage_differences)

    # Print maximum percentage difference for each component, excluding specific nodes
    for component in ['voltA_mag', 'voltB_mag', 'voltC_mag']:
        filtered_percentages = percentage_differences[~percentage_differences['node_name'].str.contains('_tm_|_tn_')]
        max_diff = filtered_percentages[f'percent_delta_{component}'].max()
        print(f"Maximum percentage difference for {component} (excluding _tm_ and _tn_ nodes): {max_diff:.2f}%")

    # Plotting the errors for each phase on the same graph using Plotly
    fig = go.Figure()

    fig.add_trace(go.Scatter(x=filtered_percentages['node_name'], y=filtered_percentages['percent_delta_voltA_mag'],
                             mode='lines+markers', name='Phase A Errors', marker=dict(symbol='circle')))
    fig.add_trace(go.Scatter(x=filtered_percentages['node_name'], y=filtered_percentages['percent_delta_voltB_mag'],
                             mode='lines+markers', name='Phase B Errors', marker=dict(symbol='x')))
    fig.add_trace(go.Scatter(x=filtered_percentages['node_name'], y=filtered_percentages['percent_delta_voltC_mag'],
                             mode='lines+markers', name='Phase C Errors', marker=dict(symbol='square')))

    # Adding labels and title
    fig.update_layout(
        title='Percentage Errors on Nodes for Different Phases (Filtered)',
        xaxis_title='Nodes',
        yaxis_title='Percentage Error',
        xaxis=dict(tickangle=-90),
        legend=dict(title='Phases'),
        template='plotly_white'
    )

    # Display the interactive plot
    fig.show()

if __name__ == "__main__":
    main()
