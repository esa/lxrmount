import java.math.*;
import java.io.*;

//import org.apache.commons.math.ode.*;
//import org.apache.commons.math.ode.nonstiff.*;
//import org.apache.commons.math.geometry.euclidean.threed.Vector3D;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
//import org.apache.commons.math.geometry.euclidean.threed.Rotation;
//import org.apache.commons.math.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
//import org.orekit.bodies.CelestialBody;
//import org.orekit.bodies.BodyShape;
//import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.errors.OrekitException;
import org.orekit.frames.Frame;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.FramesFactory;
//import org.orekit.frames.SpacecraftFrame;
//import org.orekit.frames.Transform;
//import org.orekit.orbits.CircularOrbit;
//import org.orekit.orbits.Orbit;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.Propagator;
//import org.orekit.propagation.BoundedPropagator;
import org.orekit.propagation.SpacecraftState;
//import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.analytical.tle.*;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.DateComponents;
import org.orekit.time.TimeComponents;
import org.orekit.time.TimeScalesFactory;
import org.orekit.time.TimeScale;
import org.orekit.utils.PVCoordinatesProvider;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;
//import org.orekit.forces.SphericalSpacecraft;
//import org.orekit.forces.radiation.*;
import org.orekit.data.DirectoryCrawler;
import org.orekit.data.DataContext;

public class TrackPV {

    public static void main(String[] args) {
	File orekitData = new File("/home/marek/orekit/orekit-data");
	DataContext.getDefault().getDataProvidersManager()
	    .addProvider(new DirectoryCrawler(orekitData));
	
	TLE sat = new
	    //TLE(/* W-CUBE 2023-03-09 */
	    //"1 48965U 21059CQ  23067.61569828  .00020605  00000-0  10330-2 0  9994",
	    //	"2 48965  97.5835 200.8029 0011242 255.1858 104.8130 15.17321954 93913"
	    TLE(/* ISS (ZARYA) -- ARISS 2023-05-24 */
		"1 25544U 98067A   23145.32602431  .00014206  00000-0  25493-3 0  9995",
		"2 25544  51.6413  81.8570 0005396  18.1967 120.0030 15.50160974398246"
		);
	
	Frame inertialFrame = FramesFactory.getEME2000();
	Frame ITRF = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
	OneAxisEllipsoid earth = new
	    OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS, 
			     Constants.WGS84_EARTH_FLATTENING, 
			     ITRF);
	
	GeodeticPoint station =
	    new GeodeticPoint(/* mlyn Lange Voort, Oegstgeest */
			      52.1868056 * Math.PI/180.0,
			      4.4730814 * Math.PI/180.0,
			      -3.0);
	TopocentricFrame station_frame = new TopocentricFrame(earth, station, "NL");

	TLEPropagator propagator = TLEPropagator.selectExtrapolator(sat);

	TimeScale utc = TimeScalesFactory.getUTC();
	AbsoluteDate t0 = new AbsoluteDate(args[0], utc);
	double t_end = Double.parseDouble(args[1]);
	double dt = 1.0;
	System.out.println("# t0=" + t0);
	for (double t = 0.0; t < t_end; t += dt) {
	    AbsoluteDate tx = t0.shiftedBy(t);
	    PVCoordinates pv = propagator.getPVCoordinates(tx, station_frame);
	    //System.out.println(pv);
	    Vector3D p = pv.getPosition(), v = pv.getVelocity();
	    System.out.println(t + "\t" +
			       p.getX() + "\t" + p.getY() + "\t" + p.getZ() + "\t" +
			       v.getX() + "\t" + v.getY() + "\t" + v.getZ());
			       
	}
    }
}
