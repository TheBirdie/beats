package edu.berkeley.path.beats.test.simulator;

import static org.junit.Assert.assertEquals;

import org.apache.log4j.Logger;
import org.junit.Test;

import edu.berkeley.path.beats.simulator.nodeBeahavior.Node_FlowSolver_Symmetric;

public class NodeSymmetricTest {

	private static Logger logger = Logger.getLogger(NodeSymmetricTest.class);

	private static final double EPSILON = 1E-7;

	@Test
	public void test_simple_connection () {
		logger.info("simple connection");
		Problem problem = new Problem(1, 1);
		double [] demand = {0, 0, 1, 1, 2};
		double [] supply = {0, 1, 0, 2, 1};
		for (int i = 0; i < Math.min(demand.length, supply.length); ++i) {
			problem.demand[0][0] = demand[i];
			problem.supply[0] = supply[i];
			problem.priority[0] = demand[i] + .5;
			problem.solve();
			logger.info("test case " + i + ": flow=" + problem.flow[0][0]);
			assertEquals(Math.min(demand[i], supply[i]), problem.flow[0][0], EPSILON);
		}
	}

	@Test
	public void test_merge () {
		logger.info("merge");
		Problem problem = new Problem(2, 1);
		double [][] demand = {{0, 0}, {0, 1}, {1, 0}, {1, 1}, {.5, 1.2}, {.4, .3}, {.7, .8}, {1.1, .7}};
		double [] supply = {1.5, 1.5, .5, 1, 1.5, 1, 1, 1};
		double [][] priority = {{1, 2}, {1, 2}, {1, 2}, {1, 2}, {1, 2}, {1, 0}, {1, 0}, {1, 0}};
		for (int i = 0; i < Math.min(demand.length, Math.min(supply.length, priority.length)); ++i) {
			for (int n = 0; n < 2; ++n) {
				problem.demand[n][0] = demand[i][n];
				problem.priority[n] = priority[i][n];
			}
			problem.supply[0] = supply[i];
			problem.solve();
			logger.info("test case " + i + ": flows=[" + problem.flow[0][0] + ", " + problem.flow[1][0] + "]");
			double [] q = solve_merge(demand[i], supply[i], problem.priority);
			for (int n = 0; n < 2; ++n)
				assertEquals(q[n], problem.flow[n][0], EPSILON);
		}
	}

	private static double [] solve_merge(double [] s, double r, double [] p) {
		if (2 != s.length || 2 != p.length) return null;
		if (s[0] + s[1] <= r) return s;
		else {
			double sum_p = 0;
			for (int i = 0; i < p.length; ++i)
				sum_p += p[i];
			double [] q = new double[2];
			if (0 >= sum_p) {
				logger.warn("both priorities are zero");
				q[0] = Math.min(s[0], r);
				q[1] = Math.min(s[1], r - q[0]);
			} else {
				double [] d = {p[0] / sum_p, p[1] / sum_p};
				if (s[0] <= d[0] * r) {
					q[0] = s[0];
					q[1] = r - s[0];
				} else if (s[1] < d[1] * r) {
					q[1] = s[1];
					q[0] = r - s[1];
				} else
					for (int i = 0; i < d.length; ++i)
						q[i] = d[i] * r;
			}
			return q;
		}
	}

	@Test
	public void test_diverge () {
		logger.info("diverge");
		Problem problem = new Problem(1, 2);
		double [][] demand = {{0, 0}, {0, 1}, {1, 0}, {1, 1}, {.5, 1.2}};
		double [][] supply = {{1, 1}, {1, 0}, {.5, 0}, {.5, .8}, {0, 0}};
		for (int i = 0; i < Math.min(demand.length, supply.length); ++i) {
			problem.demand[0] = demand[i];
			problem.supply = supply[i];
			problem.priority[0] = demand[i][0] + demand[i][1] + .5;
			problem.solve();
			logger.info("test case " + i + ": flows=[" + problem.flow[0][0] + ", " + problem.flow[0][1] + "]");
			double [] q = solve_diverge(demand[i], supply[i]);
			for (int n = 0; n < 2; ++n)
				assertEquals(q[n], problem.flow[0][n], EPSILON);
		}
	}

	private static double [] solve_diverge(double [] s, double r []) {
		if (2 != s.length || 2 != r.length) return null;
		double [] q = new double[2];
		double reduction = 1.0;
		for (int n = 0; n < 2; ++n)
			if (reduction * s[n] > r[n]) reduction = r[n] / s[n];
		for (int n = 0; n < 2; ++n)
			q[n] = reduction * s[n];
		return q;
	}

	private static class Problem {
		double [][] demand;
		double [] supply;
		double [] priority;
		double [][] flow;

		Node_FlowSolver_Symmetric.NodeModel model;

		public Problem(int nIn, int nOut) {
			demand = new double[nIn][nOut];
			supply = new double[nOut];
			priority = new double[nIn];
			flow = new double[nIn][nOut];

			model = new Node_FlowSolver_Symmetric.NodeModel(nIn, nOut);
		}

		public void solve() {
			model.solve(demand, supply, priority, flow);
		}
	}
}
