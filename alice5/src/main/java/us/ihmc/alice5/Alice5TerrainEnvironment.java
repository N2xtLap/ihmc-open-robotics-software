package us.ihmc.alice5;

import java.util.Random;

import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

/**
 * SIM-EXT T1 uneven-terrain environments for blind walking robustness runs.
 *
 * Types (selected via -Dalice5.terrain=):
 * - "random": 20 cm grid cells with tops uniformly distributed in [-3 cm, +3 cm] (seeded via
 *   -Dalice5.terrainSeed). Field spans x in [0.4, 4.6], y in [-1.1, 1.1]; flat start zone behind.
 * - "ramp": 8 deg straight ramp from x=0.5 to x=3.5 (rise ~0.42 m) with a flat plateau after.
 * - "steps": 4 cm elevation changes at x=1.0 (up), x=2.0 (down), x=3.0 (up) - 1 m treads.
 *
 * The combined terrain resolves overlapping objects by max height, so negative random cells sit on
 * a -10 cm sub-base with no z=0 ground beneath the field. CSG footstep z stays blind (no height map
 * is handed to the stepping plugin); only the demo's fall check reads ground truth via heightAt.
 */
public class Alice5TerrainEnvironment implements CommonAvatarEnvironmentInterface
{
   public static final double RANDOM_CELL_SIZE = 0.2;
   public static final double RANDOM_AMPLITUDE = 0.03;
   public static final double RAMP_ANGLE_DEG = 8.0;
   public static final double STEP_HEIGHT = 0.04;

   private final CombinedTerrainObject3D terrain;

   public Alice5TerrainEnvironment(String type, long seed)
   {
      terrain = new CombinedTerrainObject3D("alice5Terrain_" + type);

      switch (type)
      {
         case "random":
         {
            // start zone + side aprons at z=0, none under the cell field (negative cells must win)
            terrain.addBox(-10.0, -10.0, 0.4, 10.0, -0.05, 0.0);
            terrain.addBox(0.4, -10.0, 10.0, -1.1, -0.05, 0.0);
            terrain.addBox(0.4, 1.1, 10.0, 10.0, -0.05, 0.0);
            terrain.addBox(4.6, -1.1, 10.0, 1.1, -0.05, 0.0);
            Random random = new Random(seed);
            for (double x = 0.4; x < 4.6 - 1e-9; x += RANDOM_CELL_SIZE)
            {
               for (double y = -1.1; y < 1.1 - 1e-9; y += RANDOM_CELL_SIZE)
               {
                  double top = (2.0 * random.nextDouble() - 1.0) * RANDOM_AMPLITUDE;
                  terrain.addBox(x, y, x + RANDOM_CELL_SIZE, y + RANDOM_CELL_SIZE, -0.10, top);
               }
            }
            break;
         }
         case "ramp":
         {
            double rampStart = 0.5;
            double rampEnd = 3.5;
            double rise = (rampEnd - rampStart) * Math.tan(Math.toRadians(RAMP_ANGLE_DEG));
            terrain.addBox(-10.0, -10.0, rampStart, 10.0, -0.05, 0.0);
            terrain.addRamp(rampStart, -1.5, rampEnd, 1.5, 0.0, rise);
            terrain.addBox(rampEnd, -1.5, 10.0, 1.5, rise - 0.05, rise);
            break;
         }
         case "steps":
         {
            terrain.addBox(-10.0, -10.0, 1.0, 10.0, -0.05, 0.0);
            terrain.addBox(1.0, -1.5, 2.0, 1.5, -0.05, STEP_HEIGHT); // up 4 cm
            terrain.addBox(2.0, -1.5, 3.0, 1.5, -0.05, 0.0);         // down 4 cm
            terrain.addBox(3.0, -1.5, 10.0, 1.5, -0.05, STEP_HEIGHT); // up 4 cm
            terrain.addBox(2.0, -10.0, 10.0, -1.5, -0.05, 0.0);      // side aprons
            terrain.addBox(2.0, 1.5, 10.0, 10.0, -0.05, 0.0);
            break;
         }
         default:
            throw new IllegalArgumentException("unknown terrain type: " + type);
      }
   }

   /** Demo entry point: -Dalice5.terrain=flat|random|ramp|steps, -Dalice5.terrainSeed=N. */
   public static CommonAvatarEnvironmentInterface fromSystemProperties()
   {
      String type = System.getProperty("alice5.terrain", "flat");
      if (type.equals("flat"))
         return new FlatGroundEnvironment();
      long seed = Long.parseLong(System.getProperty("alice5.terrainSeed", "1"));
      return new Alice5TerrainEnvironment(type, seed);
   }

   /** Ground-truth terrain height (max over overlapping objects) — used by the demo fall check. */
   public double heightAt(double x, double y)
   {
      return terrain.heightAt(x, y, 10.0);
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return terrain;
   }
}
