
#include "EGPlanner/energy/contactEnergy.h"
#include "robot.h"
#include "grasp.h"
#include "debug.h"
#include "world.h"
#include "contact/virtualContact.h"

////For all points sampled in the convex hull of the virtual contacts,
////check whether it is inside the object
////First can check whether 000 is inside the body??
//double
//ContactEnergy::energy() const
//{
//  double ratio = 0.1;
//
//  if (mContactType == CONTACT_LIVE && mType != ENERGY_AUTOGRASP_QUALITY && mType !=
//      ENERGY_STRICT_AUTOGRASP){
//    mHand->getWorld()->findVirtualContacts(mHand, mObject);
//    DBGP("Live contacts computation");
//  }
//
//  mHand->getGrasp()->collectVirtualContacts();
//  int num_contact = mHand->getGrasp()->getNumContacts();
//
//  VirtualContact *contact;
//  int num_inside = 0;
//  int num_sample = 100;
//  double pos_sample[3];
//  double pos_tmp[3];
//  double alpha[num_contact];
//  std::fill_n(alpha, num_contact, 1);
//  double weight[num_contact];
//  position loc_tmp;
//
//  for (int i = 0; i < num_sample; i++){
//
//    gsl_ran_dirichlet(rand_seed, num_contact, alpha, weight);
//
//    for (int k = 0; k < 3; k++){
//  pos_sample[k] = 0.0;
//    }
//    for (int j = 0; j < num_contact; j++){
//      contact = (VirtualContact *)mHand->getGrasp()->getContact(j);
//      contact->getWorldLocation().get(pos_tmp);
//      for (int k = 0; k < 3; k++){
//  pos_sample[k] += pos_tmp[k] * weight[j];
//      }
//    }
//
//    loc_tmp.set(pos_sample[0], pos_sample[1], pos_sample[2]);
//    if (mObject->getWorld()->isPointInBodyOfWorld(loc_tmp, mObject)){
//        num_inside ++;
//    }
//  }

//  //printf("!!!Number of inside is %d!!!\n", num_inside);
//  return (- 0.01 * num_inside);
//}
//
double ContactEnergy::energy() const
{
  /*
  this is if we don't want to use pre-specified contacts, but rather the closest points between each link
  and the object all iterations. Not necessarily needed for contact energy, but useful for GQ energy
  */
  if (mContactType == CONTACT_LIVE && mType != ENERGY_AUTOGRASP_QUALITY && mType != ENERGY_STRICT_AUTOGRASP)
  {
    mHand->getWorld()->findVirtualContacts(mHand, mObject);
    DBGP("Live contacts computation");
  }
 
  mHand->getGrasp()->collectVirtualContacts();

  //DBGP("Contact energy computation")
  //average error per contact
  VirtualContact *contact;

  // Start checking inside
  double ratio = 0.1;

  int num_contact = mHand->getGrasp()->getNumContacts();

  int num_inside = 0;
  int num_sample = 500;
  double pos_sample[3];
  double pos_tmp[3];
  double alpha[num_contact];
  std::fill_n(alpha, num_contact, 1);
  double weight[num_contact];
  position loc_tmp;

  for (int i = 0; i < num_sample; i++){

    gsl_ran_dirichlet(rand_seed, num_contact, alpha, weight);

    for (int k = 0; k < 3; k++){
  pos_sample[k] = 0.0;
    }
    for (int j = 0; j < num_contact; j++){
      contact = (VirtualContact *)mHand->getGrasp()->getContact(j);
      contact->getWorldLocation().get(pos_tmp);
      for (int k = 0; k < 3; k++){
  pos_sample[k] += pos_tmp[k] * weight[j];
      }
    }

    loc_tmp.set(pos_sample[0], pos_sample[1], pos_sample[2]);
    if (mObject->getWorld()->isPointInBodyOfWorld(loc_tmp, mObject)){
        num_inside ++;
    }
  }
  double ratio_test = double(num_inside) / num_sample;
  // End checking inside
 
  vec3 p, n, cn;
  double totalError = 0;
  for (int i = 0; i < mHand->getGrasp()->getNumContacts(); i++)
  {
    contact = (VirtualContact *)mHand->getGrasp()->getContact(i);
    contact->getObjectDistanceAndNormal(mObject, &p, NULL);
    double dist = p.len();
 
    //this should never happen anymore since we're never inside the object
    //if ( (-1.0 * p) % n < 0) dist = -dist;
 
    //BEST WORKING VERSION, strangely enough
    totalError += fabs(dist);
 
    //let's try this some more
    //totalError += distanceFunction(dist);
    //cn = -1.0 * contact->getWorldNormal();
 
    //Try simply using the previous distance as the metric
    //new version
    //cn = contact->getWorldNormal();
    //n = normalise(p);
    //double d = 1 - cn % n;
    //totalError += d * 100.0 / 2.0;
  }
 
  totalError /= mHand->getGrasp()->getNumContacts();
 
  //DBGP("Contact energy: " << totalError);
  //return totalError;
  return totalError - ratio_test * 100;
} 
