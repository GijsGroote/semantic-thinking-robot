
import plotly.graph_objs as go
import numpy as np
def main():


# Create data
    x = np.arange(0, 10, 0.1)
    y1 = np.sin(x)
    y2 = np.cos(x)

# Create figure and add traces
    fig = go.Figure()

    fig.add_trace(
        go.Scatter(x=x, y=y1, mode='lines', name='Sin(x)')
    )

    fig.add_trace(
        go.Scatter(x=x, y=y2, mode='lines', name='Cos(x)')
    )

    fig.add_trace(
        go.Scatter(x=x, y=y1, fill='tonexty', fillcolor='rgba(0,100,80,0.2)', line_color='rgba(255,255,255,0)', showlegend=False)
    )

    fig.add_trace(
        go.Scatter(x=x, y=y2, fill='tonexty', fillcolor='rgba(0,176,246,0.2)', line_color='rgba(255,255,255,0)', showlegend=False)
    )

# Update layout
    fig.update_layout(
        title='Sin(x) and Cos(x)',
        xaxis_title='x',
        yaxis_title='y',
        legend=dict(
            yanchor="top",
            y=0.99,
            xanchor="left",
            x=0.01
        )
    )

# Show plot
    fig.show()

if __name__ == '__main__':
    main()
