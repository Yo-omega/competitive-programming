
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <math.h>

int calculate_thrust(int angle, int previous_thrust)
{
	int abs_angle = abs(angle);
	int thrust;

	if (abs_angle < 10)
	{
		thrust = 100; // Full speed when almost aligned
	}
	else if (abs_angle < 45)
	{
		thrust = 100 - (20 * abs_angle / 45); // Small reduction
	}
	else if (abs_angle < 90)
	{
		thrust = 80 - (30 * (abs_angle - 45) / 45); // Moderate reduction
	}
	else if (abs_angle < 160)
	{
		thrust = 50 - (20 * (abs_angle - 90) / 70); // Strong reduction
	}
	else
	{
		thrust = 30 - (10 * (abs_angle - 160) / 20); // Extreme turn, slow down smoothly
	}

	// **Momentum Adjustment** (Smooth speed transitions)
	int estimated_speed = (int)(0.8 * previous_thrust + 0.2 * thrust);
	if (estimated_speed > 200)
	{
		thrust = thrust * 0.7; // Reduce thrust by 30% if moving fast
	}
	else if (estimated_speed > 100)
	{
		thrust = thrust * 0.85; // Reduce thrust by 15% at medium speed
	}

	// Ensure thrust is within valid range
	if (thrust < 20)
		thrust = 20;
	if (thrust > 100)
		thrust = 100;

	return thrust;
}

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

int main()
{
	// game loop
	int thrust = 0;
	while (1)
	{
		int x;
		int y;
		// x position of the next check point
		int next_checkpoint_x;
		// y position of the next check point
		int next_checkpoint_y;
		// distance to the next checkpoint
		int next_checkpoint_dist;
		// angle between your pod orientation and the direction of the next checkpoint
		int next_checkpoint_angle;
		scanf("%d%d%d%d%d%d", &x, &y, &next_checkpoint_x, &next_checkpoint_y, &next_checkpoint_dist, &next_checkpoint_angle);
		int opponent_x;
		int opponent_y;
		scanf("%d%d", &opponent_x, &opponent_y);

		// Write an action using printf(). DON'T FORGET THE TRAILING \n
		// To debug: fprintf(stderr, "Debug messages...\n");

		// You have to output the target position
		// followed by the power (0 <= thrust <= 100)
		// i.e.: "x y thrust"
		fprintf(stderr, "next_checkpoint_dist: %d\n", next_checkpoint_dist);
		fprintf(stderr, "next_checkpoint_x: %d\n", next_checkpoint_x);
		fprintf(stderr, "next_checkpoint_y: %d\n", next_checkpoint_y);
		thrust = calculate_thrust(next_checkpoint_angle, thrust);
		if (next_checkpoint_dist < 10000 && next_checkpoint_dist > 3000 && abs(next_checkpoint_angle) < 40)
			printf("%d %d BOOST\n", next_checkpoint_x, next_checkpoint_y);
		else if (next_checkpoint_dist < 2500)
			printf("%d %d %d\n", next_checkpoint_x, next_checkpoint_y, abs(thrust - 20));
		else
			printf("%d %d %d\n", next_checkpoint_x, next_checkpoint_y, thrust);
	}

	return 0;
}